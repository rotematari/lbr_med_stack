#define MCAP_IMPLEMENTATION
#include "lbr_acquisition/offline_writer.hpp"
#include "lbr_acquisition/incremental_writer.hpp"
#include "lbr_acquisition/readback.hpp"
#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <google/protobuf/descriptor.pb.h>
#include <iomanip>
#include <linux/fs.h>
#include <mcap/reader.hpp>
#include <mcap/writer.hpp>
#include <memory>
#include <openssl/rand.h>
#include <openssl/sha.h>
#include <set>
#include <sstream>
#include <sys/syscall.h>
#include <unistd.h>

namespace lbr_acquisition {
ssize_t FileOps::write(int fd, const void *data, size_t n) {
  return ::write(fd, data, n);
}
int FileOps::close(int fd) { return ::close(fd); }
namespace {
constexpr size_t file_limit = 16 * 1024 * 1024;
class Sink final : public mcap::IWritable {
  int fd_ = -1;
  FileOps &io_;
  uint64_t size_ = 0;
  size_t limit_;
  int *close_error_;

public:
  Sink(const std::filesystem::path &path, FileOps &io,
       size_t limit = file_limit, int *close_error = nullptr)
      : io_(io), limit_(limit), close_error_(close_error) {
    fd_ = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
    if (fd_ < 0)
      throw IoError("exclusive open: " + std::string(std::strerror(errno)));
  }
  ~Sink() override { close_once(); }
  int close_once() noexcept {
    if (fd_ < 0)
      return 0;
    const int fd = fd_;
    fd_ = -1;
    int error = 0;
    try {
      if (io_.close(fd) != 0)
        error = errno ? errno : EIO;
    } catch (...) {
      error = EIO;
    }
    if (error && close_error_ && !*close_error_)
      *close_error_ = error;
    return error;
  }
  uint64_t size() const override { return size_; }
  void handleWrite(const std::byte *p, uint64_t n) override {
    if (fd_ < 0)
      throw IoError("write after close");
    if (n > limit_ - size_)
      throw IoError("output size limit");
    while (n) {
      auto accepted = io_.write(fd_, p, n);
      if (accepted < 0 && errno == EINTR)
        continue;
      if (accepted < 0)
        throw IoError("write failed: " + std::string(std::strerror(errno)));
      if (accepted == 0 || uint64_t(accepted) > n)
        throw IoError("write made no progress or returned invalid size");
      n -= accepted;
      p += accepted;
      size_ += accepted;
    }
  }
  // Unbuffered POSIX write has no user-space buffer to flush. This is not
  // fsync.
  void end() override {
    const int error = close_once();
    if (error)
      throw IoError("close failed: " + std::string(std::strerror(error)));
  }
  void text(const std::string &s) {
    write(reinterpret_cast<const std::byte *>(s.data()), s.size());
  }
};
std::string read_bounded(const std::filesystem::path &path, size_t bound) {
  std::ifstream f(path, std::ios::binary);
  if (!f)
    throw IoError("cannot read " + path.string());
  std::string bytes(bound + 1, '\0');
  f.read(bytes.data(), bytes.size());
  bytes.resize(f.gcount());
  if (f.bad())
    throw IoError("read failure");
  if (bytes.size() > bound)
    throw InvalidInput("descriptor exceeds bound");
  return bytes;
}
std::string quote(const std::string &s) {
  std::ostringstream o;
  o << '"';
  for (unsigned char c : s) {
    if (c == '"' || c == '\\')
      o << '\\' << c;
    else if (c < 32)
      o << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c);
    else
      o << c;
  }
  o << '"';
  return o.str();
}
std::string list(const std::vector<std::string> &issues) {
  std::string s = "[";
  for (auto &issue : issues) {
    if (s.size() > 1)
      s += ",";
    s += quote(issue);
  }
  return s + "]";
}
void text_file(const std::filesystem::path &p, const std::string &data,
               FileOps &io, int *close_error = nullptr,
               size_t limit = file_limit) {
  Sink f(p, io, limit, close_error);
  f.text(data);
  f.end();
}
void promote_noreplace(const std::filesystem::path &partial,
                       const std::filesystem::path &final) {
  if (::syscall(SYS_renameat2, AT_FDCWD, partial.c_str(), AT_FDCWD,
                final.c_str(), RENAME_NOREPLACE) != 0)
    throw IoError("final rename: " + std::string(std::strerror(errno)));
}
std::string digest(const std::string &bytes) {
  unsigned char hash[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char *>(bytes.data()), bytes.size(),
         hash);
  std::ostringstream o;
  for (auto c : hash)
    o << std::hex << std::setw(2) << std::setfill('0') << unsigned(c);
  return o.str();
}
std::string quality(bool complete, bool valid,
                    const std::vector<std::string> &issues) {
  return "{\"container_complete\":" + std::string(complete ? "true" : "false") +
         ",\"data_valid\":" + (valid ? "true" : "false") +
         ",\"timing_acceptance\":\"not_evaluated\",\"sysid_suitability\":\"not_"
         "evaluated\",\"issues\":" +
         list(issues) +
         ",\"finalization\":" + quote(complete ? "complete" : "failed") + "}\n";
}
struct EventValidator {
  std::set<uint64_t> states;
  uint64_t last_state = 0, last_submission = 0, last_host = 0;
  bool first = true;
  std::vector<std::string> issues;
  uint64_t count = 0;
  CaptureLimits limits{};
  bool latest_state_only = false;
  void issue(const std::string &message) {
    // These are finite diagnostic categories, not a per-record history.
    if (!latest_state_only || std::find(issues.begin(), issues.end(), message) == issues.end())
      issues.push_back(message);
  }
  void accept(const OfflineEvent &event) {
    if (++count > limits.events) throw InvalidInput("event limit");
    std::visit(
        [&](const auto &m) {
          using T = std::decay_t<decltype(m)>;
          if (!m.IsInitialized())
            throw InvalidInput("missing required field");
          if (m.schema_version() != 1 ||
              m.GetReflection()->GetUnknownFields(m).field_count())
            throw InvalidInput("unsupported schema");
          if (m.ByteSizeLong() > 4096)
            throw InvalidInput("payload limit");
          if (!first && m.host_monotonic_ns() < last_host)
            issue("disordered host timestamp");
          first = false;
          last_host = m.host_monotonic_ns();
          const google::protobuf::RepeatedField<double> *q;
          if constexpr (std::is_same_v<T, lbr::acquisition::v1::State>) {
            if (m.state_id() <= last_state)
              throw InvalidInput("state IDs must increase and be positive");
            last_state = m.state_id();
            if (!latest_state_only) states.insert(last_state);
            q = &m.q_measured_rad();
            const std::pair<const char *, const google::protobuf::RepeatedField<double> *> torques[] = {
                {"tau_measured_nm", &m.tau_measured_nm()},
                {"tau_commanded_nm", &m.tau_commanded_nm()},
                {"tau_external_estimated_nm", &m.tau_external_estimated_nm()}};
            for (const auto &[name, values] : torques) {
              if (values->size() != 0 && values->size() != 7)
                throw InvalidInput(std::string(name) + ": expected zero or seven joints");
              if (std::any_of(values->begin(), values->end(),
                              [](double v) { return !std::isfinite(v); }))
                issue(std::string("nonfinite ") + name);
            }
            if (m.has_controller_sec() != m.has_controller_nsec())
              throw InvalidInput("one-sided controller clock");
            if (m.has_controller_nsec() && m.controller_nsec() >= 1000000000)
              issue("controller nanoseconds out of range");
            if (m.has_reported_sample_period_s() &&
                (!std::isfinite(m.reported_sample_period_s()) ||
                 m.reported_sample_period_s() <= 0))
              issue("invalid sample period");
            if ((m.has_session_state() && m.session_state() > 4) ||
                (m.has_connection_quality() && m.connection_quality() > 3) ||
                (m.has_control_mode() && m.control_mode() > 3) ||
                (m.has_client_command_mode() && m.client_command_mode() > 3))
              issue("unknown SDK code");
          } else {
            if (m.submission_id() <= last_submission)
              throw InvalidInput(
                  "submission IDs must increase and be positive");
            if (latest_state_only ? (!last_state || m.state_id() != last_state) : !states.count(m.state_id()))
              throw InvalidInput("submission references unseen state");
            last_submission = m.submission_id();
            q = &m.q_submitted_rad();
          }
          if (q->size() != 7)
            throw InvalidInput("expected seven joints");
          uint32_t mask = 0;
          for (int i = 0; i < 7; ++i)
            if (!std::isfinite(q->Get(i)))
              mask |= 1U << i;
          if (mask != m.nonfinite_joint_mask())
            throw InvalidInput("nonfinite mask mismatch");
          if (mask)
            issue("nonfinite joints");
        },
        event);
  }
};
} // namespace

std::vector<std::string>
validate_events(const std::vector<OfflineEvent> &events) {
  EventValidator validator;
  for (const auto &event : events) validator.accept(event);
  return validator.issues;
}

std::vector<OfflineEvent> synthetic_events() {
  using lbr::acquisition::v1::State;
  using lbr::acquisition::v1::Submission;
  auto state = [](uint64_t id, uint64_t time, std::array<double, 7> q) {
    State s;
    s.set_schema_version(1);
    s.set_state_id(id);
    s.set_host_monotonic_ns(time);
    s.set_nonfinite_joint_mask(0);
    s.set_reported_sample_period_s(.001);
    for (double v : q)
      s.add_q_measured_rad(v);
    return s;
  };
  auto sub = [](uint64_t id, uint64_t parent, uint64_t time,
                std::array<double, 7> q) {
    Submission s;
    s.set_schema_version(1);
    s.set_submission_id(id);
    s.set_state_id(parent);
    s.set_host_monotonic_ns(time);
    s.set_nonfinite_joint_mask(0);
    for (double v : q)
      s.add_q_submitted_rad(v);
    return s;
  };
  auto a = state(100, 9007199254740992ULL,
                 {.125, -0.0, -.25, .375, -.5, .625, -.75});
  auto b = state(101, 9007199254740993ULL,
                 {.25, .125, -.375, .5, -.625, .75, -.875});
  b.set_controller_sec(0);
  b.set_controller_nsec(0);
  auto c =
      state(102, 9007199255740993ULL, {.375, .25, -.5, .625, -.75, .875, -1.0});
  c.set_controller_sec(0);
  c.set_controller_nsec(1000000);
  return {a, b,
          sub(200, 101, 9007199254740994ULL,
              {.5, -.125, .25, -.375, .5, -.625, .75}),
          c,
          sub(201, 102, 9007199255740994ULL,
              {.625, -.25, .375, -.5, .625, -.75, .875})};
}

struct IncrementalFixtureWriter::Impl {
  std::filesystem::path dir;
  FileOps &io;
  const CaptureSource source;
  const CaptureLimits limits;
  std::string descriptor;
  EventValidator validator;
  int cleanup_close_error = 0;
  std::unique_ptr<Sink> log, sink;
  mcap::McapWriter writer;
  std::unique_ptr<mcap::Schema> state, submission;
  std::unique_ptr<mcap::Channel> sc, cc;
  bool opened = false, finished = false, failed = false;
  bool seen_state = false, seen_submission = false;

  Impl(std::filesystem::path directory, const std::filesystem::path &desc,
       FileOps &file_io, CaptureSource capture_source, CaptureLimits budgets, bool latest_only)
      : dir(std::move(directory)), io(file_io), source(capture_source), limits(checked_limits(budgets)),
        descriptor(read_bounded(desc, 65536)) {
    validator.limits = limits;
    validator.latest_state_only = latest_only;
    capture_source_name(source); // Reject unknown values before creating output.
    if (descriptor != read_bounded(LBR_BUILD_DESCRIPTOR, 65536))
      throw InvalidInput("descriptor differs from locally generated schema");
    std::error_code ec;
    if (!std::filesystem::create_directory(dir, ec)) {
      if (!ec || ec == std::errc::file_exists)
        throw ExistingOutput("output directory already exists");
      throw IoError("create output directory: " + ec.message());
    }
    try {
      prepare();
    } catch (...) {
      poison(std::current_exception());
      throw;
    }
  }
  void prepare() {
    unsigned char random_id[16];
    if (RAND_bytes(random_id, sizeof(random_id)) != 1)
      throw IoError("run ID generation failed");
    std::ostringstream id_stream;
    for (auto c : random_id)
      id_stream << std::hex << std::setw(2) << std::setfill('0') << unsigned(c);
    const auto run_id = id_stream.str();
    text_file(
        dir / "manifest.json",
        "{\"schema_version\":1,\"profile\":" + quote(capture_profile(source)) +
            ",\"run_id\":" +
            quote(run_id) +
            ",\"source\":" + quote(capture_source_name(source)) +
            ",\"joint_names\":[\"lbr_A1\",\"lbr_A2\","
            "\"lbr_A3\",\"lbr_A4\",\"lbr_A5\",\"lbr_A6\",\"lbr_A7\"],\"units\":"
            "{\"position\":\"rad\",\"torque\":\"Nm\"},\"clocks\":{\"host_monotonic_ns\":" +
            quote(source == CaptureSource::Synthetic
                      ? "synthetic host monotonic nanoseconds"
                      : "host monotonic nanoseconds at FRI observation") +
            ",\"controller\":\"separate unsynchronized "
            "seconds and nanoseconds, "
            "optional\"},\"publish_time_available\":false,\"descriptor_"
            "sha256\":" +
            quote(digest(descriptor)) +
            ",\"software_versions\":{\"mcap_cpp\":\"2.1.3\",\"protobuf_cpp\":"
            "\"3.21.12\",\"mcap_commit\":"
            "\"00e47a1ba6901cbc0cf448cccebbc554c545c4fa\"},\"limits\":{\"file_"
            "bytes\":" + std::to_string(limits.file_bytes) + ",\"record_bytes\":1048576,\"payload_bytes\":4096,"
            "\"descriptor_bytes\":65536,\"schemas\":2,\"channels\":2,"
            "\"events\":" + std::to_string(limits.events) + ",\"manifest_bytes\":65536},\"events_jsonl\":"
            "\"not emitted by storage writer; session diagnostics may provide events.jsonl\","
            "\"power_loss_durability\":false}\n",
        io, &cleanup_close_error, 65536);

    log = std::make_unique<Sink>(dir / "terminal.log", io, file_limit,
                                 &cleanup_close_error);
    log->text(std::string("capture_source: ") + capture_source_name(source) +
              "\nwriting signals.mcap.partial\n");
    sink = std::make_unique<Sink>(dir / "signals.mcap.partial", io, limits.file_bytes,
                                  &cleanup_close_error);
    mcap::McapWriterOptions options("");
    options.library = "lbr-acquisition-offline/0.1.0 mcap/2.1.3";
    options.compression = mcap::Compression::None;
    options.chunkSize = 65536;
    options.noChunking = false;
    options.noChunkCRC = false;
    options.enableDataCRC = true;
    options.noMessageIndex = true;
    options.noChunkIndex = true;
    options.noSummary = true;
    options.noSummaryCRC = true;
    writer.open(*sink, options);
    opened = true;
    state = std::make_unique<mcap::Schema>("lbr.acquisition.v1.State",
                                            "protobuf", descriptor);
    submission = std::make_unique<mcap::Schema>(
        "lbr.acquisition.v1.Submission", "protobuf", descriptor);
    writer.addSchema(*state);
    writer.addSchema(*submission);
    mcap::KeyValueMap metadata{{"clock", "host_monotonic"},
                               {"position_units", "rad"},
                               {"torque_units", "Nm"},
                               {"joint_order", "A1,A2,A3,A4,A5,A6,A7"},
                               {"source", capture_source_name(source)}};
    sc = std::make_unique<mcap::Channel>("/lbr_acquisition/state",
                                          "protobuf", state->id, metadata);
    cc = std::make_unique<mcap::Channel>("/lbr_acquisition/submission",
                                          "protobuf", submission->id, metadata);
    writer.addChannel(*sc);
    writer.addChannel(*cc);
  }
  void poison(std::exception_ptr error) noexcept {
    if (failed || finished) return;
    failed = true;
    // open() can throw after MCAP marks itself open. Its destructor calls
    // close(), which would retry a failed write during unwinding.
    try { writer.terminate(); } catch (...) {}
    opened = false;
    if (sink) sink->close_once();
    if (log) log->close_once();
    // Keep partials. Failure metadata is best-effort and never grants success.
    try {
      std::string message = "unknown failure";
      try { if (error) std::rethrow_exception(error); }
      catch (const std::exception &e) { message = e.what(); }
      std::vector<std::string> failures{message};
      if (cleanup_close_error)
        failures.push_back("cleanup close errno=" +
                           std::to_string(cleanup_close_error));
      text_file(dir / "quality.json", quality(false, false, failures), io);
    } catch (...) {}
  }
  void append(const OfflineEvent &event, bool close_chunk) {
    if (failed || finished) throw InvalidInput("fixture writer already closed");
    try {
      validator.accept(event);
      std::visit([&](const auto &m) {
        const auto bytes = m.SerializeAsString();
        mcap::Message message;
        message.channelId = event.index() == 0 ? sc->id : cc->id;
        message.sequence = 0;
        message.logTime = m.host_monotonic_ns();
        message.publishTime = message.logTime;
        message.data = reinterpret_cast<const std::byte *>(bytes.data());
        message.dataSize = bytes.size();
        const auto result = writer.write(message);
        if (!result.ok()) throw IoError(result.message);
      }, event);
      if (close_chunk) writer.closeLastChunk();
      if (event.index() == 0) seen_state = true;
      else seen_submission = true;
    } catch (...) {
      poison(std::current_exception());
      throw;
    }
  }
  void finish(const CaptureSummary &expected) {
    if (failed || finished) throw InvalidInput("fixture writer already closed");
    try {
      if (expected.events != validator.count)
        throw InvalidInput("expected event count differs from written count");
      // Lazy MCAP definitions for unused channels are emitted exactly once.
      if (!seen_state) {
        mcap::McapWriter::write(*sink, *state);
        mcap::McapWriter::write(*sink, *sc);
      }
      if (!seen_submission) {
        mcap::McapWriter::write(*sink, *submission);
        mcap::McapWriter::write(*sink, *cc);
      }
      writer.close(); // checked sink close
      opened = false;
      verify_capture_container(dir / "signals.mcap.partial", expected, descriptor, source, limits);
      log->text(std::string("native linear CRC and value readback passed\nfinalizing ") +
                capture_source_name(source) + "; timing/sysid not evaluated\n");
      log->end();
      promote_noreplace(dir / "signals.mcap.partial", dir / "signals.mcap");
      text_file(dir / "quality.json.partial",
                quality(true, validator.issues.empty(), validator.issues),
                io, &cleanup_close_error);
      promote_noreplace(dir / "quality.json.partial", dir / "quality.json");
      finished = true;
    } catch (...) {
      poison(std::current_exception());
      throw;
    }
  }
  ~Impl() {
    if (!finished && opened) {
      try { writer.terminate(); } catch (...) {}
    }
  }
};

IncrementalFixtureWriter::IncrementalFixtureWriter(
    std::filesystem::path dir, std::filesystem::path desc, FileOps &io,
    CaptureSource source, CaptureLimits limits, bool latest_state_only)
    : impl_(std::make_unique<Impl>(std::move(dir), desc, io, source, limits, latest_state_only)) {}
IncrementalFixtureWriter::~IncrementalFixtureWriter() = default;
void IncrementalFixtureWriter::append(const OfflineEvent &e, bool flush) {
  impl_->append(e, flush);
}
void IncrementalFixtureWriter::finish(const std::vector<OfflineEvent> &expected) {
  CaptureSummary summary;
  for (const auto &event : expected) summary.observe(event);
  impl_->finish(summary);
}
void IncrementalFixtureWriter::finish_capture(const CaptureSummary &expected) {
  impl_->finish(expected);
}
const std::filesystem::path &
IncrementalFixtureWriter::output_directory() const noexcept { return impl_->dir; }

void write_fixture(const std::filesystem::path &dir,
                   const std::vector<OfflineEvent> &events,
                   const std::filesystem::path &desc) {
  FileOps io;
  write_fixture(dir, events, desc, io);
}
void write_fixture(const std::filesystem::path &dir,
                   const std::vector<OfflineEvent> &events,
                   const std::filesystem::path &desc, FileOps &io) {
  validate_events(events); // preserve pre-directory rejection
  IncrementalFixtureWriter writer(dir, desc, io);
  for (const auto &event : events) writer.append(event, false);
  writer.finish(events);
}
} // namespace lbr_acquisition
