#include "lbr_acquisition/streaming_capture_session.hpp"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/fs.h>
#include <sstream>
#include <stdexcept>
#include <sys/syscall.h>
#include <unistd.h>

namespace lbr_acquisition {
#ifdef LBR_SESSION_TEST_HOOKS
void session_test_before_stop_publish() noexcept;
#endif
namespace {
uint64_t positive_period(std::chrono::nanoseconds value) {
  if (value.count() <= 0)
    throw InvalidInput("session poll and stall periods must be positive");
  return static_cast<uint64_t>(value.count());
}
size_t diagnostic_capacity(size_t value) {
  if (value == 0 || value > 32)
    throw InvalidInput("session diagnostic capacity must be 1..32");
  return value;
}
const char *phase_name(capture::Phase phase) noexcept {
  switch (phase) {
  case capture::Phase::Ready: return "Ready";
  case capture::Phase::Armed: return "Armed";
  case capture::Phase::Recording: return "Recording";
  case capture::Phase::Stopping: return "Stopping";
  case capture::Phase::Finalizing: return "Finalizing";
  case capture::Phase::Complete: return "Complete";
  case capture::Phase::Failed: return "Failed";
  }
  return "Unknown";
}
const char *reason_name(capture::Reason reason) noexcept {
  switch (reason) {
  case capture::Reason::None: return "None";
  case capture::Reason::Manual: return "Manual";
  case capture::Reason::Duration: return "Duration";
  case capture::Reason::NoData: return "NoData";
  case capture::Reason::Overflow: return "Overflow";
  case capture::Reason::InvalidEvent: return "InvalidEvent";
  case capture::Reason::EventLimit: return "EventLimit";
  case capture::Reason::SourceUnavailable: return "SourceUnavailable";
  case capture::Reason::ClockRegression: return "ClockRegression";
  case capture::Reason::WriterFailure: return "WriterFailure";
  }
  return "Unknown";
}
void write_checked(const std::filesystem::path &path, const std::string &bytes,
                   FileOps &io) {
  if (bytes.size() > 65536) throw IoError("session diagnostic limit");
  const auto partial = path.string() + ".partial";
  const int fd = ::open(partial.c_str(), O_WRONLY | O_CREAT | O_EXCL |
                                       O_CLOEXEC | O_NOFOLLOW, 0600);
  if (fd < 0) throw IoError("diagnostic open: " + std::string(std::strerror(errno)));
  bool open = true;
  try {
    size_t offset = 0;
    while (offset < bytes.size()) {
      const ssize_t n = io.write(fd, bytes.data() + offset,
                                 bytes.size() - offset);
      if (n < 0 && errno == EINTR) continue;
      if (n < 0) throw IoError("diagnostic write: " + std::string(std::strerror(errno)));
      if (n == 0 || static_cast<size_t>(n) > bytes.size() - offset)
        throw IoError("diagnostic write made no progress");
      offset += static_cast<size_t>(n);
    }
    const int rc = io.close(fd);
    open = false; // close may report a delayed error; never retry it
    if (rc != 0) throw IoError("diagnostic close: " + std::string(std::strerror(errno)));
    if (::syscall(SYS_renameat2, AT_FDCWD, partial.c_str(), AT_FDCWD,
                  path.c_str(), RENAME_NOREPLACE) != 0)
      throw IoError("diagnostic rename: " + std::string(std::strerror(errno)));
  } catch (...) {
    if (open) { try { io.close(fd); } catch (...) {} }
    throw;
  }
}
} // namespace

uint64_t StreamingCaptureSession::now_ns() noexcept {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}
StreamingCaptureSession::StreamingCaptureSession(
    capture::Config config, std::filesystem::path directory,
    std::filesystem::path descriptor, FileOps &io,
    std::chrono::nanoseconds poll_period,
    std::chrono::nanoseconds writer_stall_budget,
    SessionThreadLauncher *launcher, size_t diagnostic_limit,
    CaptureSource source)
    : config_(config), source_(source), directory_(std::move(directory)), io_(io),
      poll_ns_(positive_period(poll_period)),
      stall_ns_(positive_period(writer_stall_budget)),
      ledger_capacity_(diagnostic_capacity(diagnostic_limit)),
      launcher_(launcher),
      recorder_(config), sink_(config.run_id, directory_, std::move(descriptor), io,
                              source_, capture_limits_for(config.max_events)) {
}
StreamingCaptureSession::~StreamingCaptureSession() {
  auto expected = capture::Reason::None;
  requested_reason_.compare_exchange_strong(expected, capture::Reason::Manual);
  join_workers(false);
}
bool StreamingCaptureSession::start() {
  std::unique_lock<std::mutex> owner(owner_mutex_);
  if (start_called_.load() || requested_reason_.load() != capture::Reason::None)
    return false;
  start_called_.store(true);
  try {
    coordinator_ = launcher_
        ? launcher_->launch([this] { coordinator_loop(); })
        : std::thread([this] { coordinator_loop(); });
    writer_ = launcher_
        ? launcher_->launch([this] { writer_loop(); })
        : std::thread([this] { writer_loop(); });
  } catch (...) {
    runtime_failed_.store(true);
    auto expected = capture::Reason::None;
    requested_reason_.compare_exchange_strong(expected, capture::Reason::Manual);
    if (!writer_.joinable()) writer_exited_.store(true);
    if (coordinator_.joinable()) coordinator_.join();
    if (writer_.joinable()) writer_.join();
    joined_.store(true);
    return false;
  }
  owner.unlock();
  std::unique_lock<std::mutex> start_lock(start_mutex_);
  start_cv_.wait(start_lock, [this] {
    return core_started_.load() || coordinator_exited_.load();
  });
  return core_started_.load();
}
capture::Offer StreamingCaptureSession::offer(const capture::Event &event,
                                              bool eligible) noexcept {
  return recorder_.offer(event, eligible);
}
bool StreamingCaptureSession::request_stop(uint64_t run_id,
                                            capture::Reason reason) noexcept {
  if (run_id != config_.run_id || reason == capture::Reason::None ||
      static_cast<uint32_t>(reason) >
          static_cast<uint32_t>(capture::Reason::WriterFailure)) return false;
#ifdef LBR_SESSION_TEST_HOOKS
  session_test_before_stop_publish();
#endif
  auto expected = capture::Reason::None;
  requested_reason_.compare_exchange_strong(expected, reason);
  return true;
}
StreamingSessionStatus StreamingCaptureSession::status() const noexcept {
  return {recorder_.status(), core_started_.load(), writer_exited_.load(),
          coordinator_exited_.load(), stalled_.load(),
          diagnostics_failed_.load(), outcome_failed_.load(), runtime_failed_.load(),
          outcome_published_.load(), joined_.load(), observed_events_.load(),
          requested_reason_.load()};
}
void StreamingCaptureSession::observe(const capture::Status &s,
                                      uint64_t now) noexcept {
  if (ledger_count_ && ledger_[ledger_count_ - 1].phase == s.phase &&
      ledger_[ledger_count_ - 1].reason == s.reason &&
      ledger_[ledger_count_ - 1].flags == s.failure_flags) return;
  if (ledger_count_ == ledger_capacity_) {
    diagnostics_failed_.store(true);
    runtime_failed_.store(true);
    if (s.reason == capture::Reason::None)
      recorder_.stop(config_.run_id, capture::Reason::Manual);
    return;
  }
  ledger_[ledger_count_++] = {now, s.phase, s.reason, s.failure_flags};
  observed_events_.store(ledger_count_);
}
void StreamingCaptureSession::coordinator_loop() noexcept {
  try {
    {
      std::lock_guard<std::mutex> start_lock(start_mutex_);
      if (!recorder_.start(config_.run_id, now_ns())) {
        runtime_failed_.store(true);
      } else {
        core_started_.store(true);
      }
    }
    start_cv_.notify_all();
    for (;;) {
      const auto now = now_ns();
      const auto requested = requested_reason_.load();
      if (requested != capture::Reason::None)
        recorder_.stop(config_.run_id, requested);
      else
        recorder_.tick(now);
      const auto seq = call_sequence_.load();
      const auto since = call_started_ns_.load();
      if (call_active_.load() && seq == call_sequence_.load() &&
          now >= since && now - since >= stall_ns_) {
        stalled_.store(true);
        // A stall closes admission if still recording. If already stopping,
        // slow drain/readback alone must not invalidate a successful finish.
        // A thrown write/finish remains a core WriterFailure in either case.
        recorder_.stop(config_.run_id, capture::Reason::WriterFailure);
      }
      observe(recorder_.status(), now);
      if (writer_exited_.load()) {
        // Startup failure can post stop after the request snapshot above, then
        // publish writer_exited. Consume that request before the coordinator exits.
        const auto final_request = requested_reason_.load();
        if (final_request != capture::Reason::None)
          recorder_.stop(config_.run_id, final_request);
        break;
      }
      std::this_thread::sleep_for(std::chrono::nanoseconds(poll_ns_));
    }
    observe(recorder_.status(), now_ns());
  } catch (...) {
    runtime_failed_.store(true);
    auto expected = capture::Reason::None;
    requested_reason_.compare_exchange_strong(expected, capture::Reason::WriterFailure);
    recorder_.stop(config_.run_id, capture::Reason::WriterFailure);
    start_cv_.notify_all();
  }
  {
    std::lock_guard<std::mutex> start_lock(start_mutex_);
    coordinator_exited_.store(true);
  }
  start_cv_.notify_all();
}
void StreamingCaptureSession::writer_loop() noexcept {
  struct TrackedWriter final : capture::Writer {
    StreamingCaptureSession &owner;
    explicit TrackedWriter(StreamingCaptureSession &s) : owner(s) {}
    void begin() noexcept {
      owner.call_started_ns_.store(StreamingCaptureSession::now_ns());
      owner.call_sequence_.fetch_add(1);
      owner.call_active_.store(true);
    }
    void end() noexcept { owner.call_active_.store(false); }
    void write(const capture::Event &e) override {
      begin();
      try { owner.sink_.write(e); } catch (...) { end(); throw; }
      end();
    }
    void finish() override {
      begin();
      try { owner.sink_.finish(); } catch (...) { end(); throw; }
      end();
    }
  } tracked(*this);
  try {
    while (!recorder_.status().writer_done) {
      if (!recorder_.writer_step(tracked))
        std::this_thread::sleep_for(std::chrono::nanoseconds(poll_ns_));
    }
  } catch (...) {
    runtime_failed_.store(true);
    auto expected = capture::Reason::None;
    requested_reason_.compare_exchange_strong(expected, capture::Reason::WriterFailure);
  }
  writer_exited_.store(true);
}
void StreamingCaptureSession::publish_diagnostics() {
  std::ostringstream events;
  for (size_t i = 0; i < ledger_count_; ++i) {
    const auto &e = ledger_[i];
    events << "{\"schema_version\":1,\"source\":\""
           << (source_ == CaptureSource::Synthetic ? "offline-recorder-session"
                                                  : capture_source_name(source_))
           << "\","
           << "\"capture_run_id\":\"" << config_.run_id << "\","
           << "\"host_monotonic_ns\":\"" << e.at_ns << "\","
           << "\"observed_phase\":\"" << phase_name(e.phase) << "\","
           << "\"reason\":\"" << reason_name(e.reason) << "\","
           << "\"failure_flags\":" << e.flags << "}\n";
  }
  write_checked(directory_ / "events.jsonl", events.str(), io_);
  const auto core = recorder_.status();
  std::ostringstream log;
  log << (source_ == CaptureSource::Synthetic
              ? "offline recorder session only; external process stdout/stderr excluded\n"
              : "FRI SDK recorder session; external process stdout/stderr excluded\n")
      << "source=" << capture_source_name(source_) << '\n'
      << "capture_run_id=" << config_.run_id << " phase=" << phase_name(core.phase)
      << " reason=" << reason_name(core.reason) << " attempted=" << core.attempted
      << " enqueued=" << core.enqueued << " written=" << core.written
      << " stalled=" << stalled_.load()
      << " runtime_failed=" << runtime_failed_.load() << '\n'
      << "written_semantics=closed_mcap_chunk_no_fsync\n";
  write_checked(directory_ / "session.log", log.str(), io_);
}
void StreamingCaptureSession::join_workers(bool publish) noexcept {
  std::lock_guard<std::mutex> owner(owner_mutex_);
  if (joined_.load() || !start_called_.load()) return;
  if (coordinator_.joinable()) coordinator_.join();
  if (writer_.joinable()) writer_.join();
  joined_.store(true);
  if (!publish) return;
  try {
    if (diagnostics_failed_.load()) throw IoError("diagnostic ledger overflow");
    publish_diagnostics();
  } catch (...) {
    diagnostics_failed_.store(true);
    runtime_failed_.store(true);
    return;
  }
  const auto core = recorder_.status();
  if (runtime_failed_.load() || !core.writer_done) return;
  try {
    write_capture_outcome(sink_, core, io_);
    outcome_published_.store(true);
  } catch (...) {
    outcome_failed_.store(true);
    runtime_failed_.store(true);
  }
}
void StreamingCaptureSession::join() { join_workers(true); }
} // namespace lbr_acquisition
