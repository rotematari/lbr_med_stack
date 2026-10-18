#include "lbr_acquisition/streaming_capture_session.hpp"
#include <cassert>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <condition_variable>
#include <mutex>
#include <string>
#include <stdexcept>
#include <thread>
#include <unistd.h>
using namespace lbr_acquisition;
namespace fs = std::filesystem;
using namespace std::chrono_literals;
template <class Predicate> static bool until(Predicate predicate) {
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) return true;
    std::this_thread::sleep_for(1ms);
  }
  return predicate();
}
int main(int argc, char **argv) {
  const bool fixtures = argc == 4 && std::string(argv[1]) == "--fixtures";
  assert(argc == 2 || fixtures);
  char pattern[] = "/tmp/acq-stream-session-XXXXXX";
  fs::path root = fixtures ? fs::path(argv[2]) : fs::path(mkdtemp(pattern));
  assert(!root.empty());
  fs::create_directories(root);
  const fs::path descriptor(fixtures ? argv[3] : argv[1]);
  FileOps io;
  bool invalid_period = false;
  try {
    StreamingCaptureSession bad({23, 1000000000ULL, 100000000ULL, 10, 8},
                                root / "bad-period", descriptor, io, 0ns, 500ms);
  } catch (const InvalidInput &) { invalid_period = true; }
  assert(invalid_period && !fs::exists(root / "bad-period"));
  {
    StreamingCaptureSession session({17, 1000000000ULL, 500000000ULL, 10, 8},
                                    root / "manual", descriptor, io, 1ms, 500ms);
    assert(session.start());
    assert(!session.start());
    assert(!session.request_stop(999));
    assert(until([&] { return session.status().capture.phase == capture::Phase::Armed; }));
    capture::Event event{};
    event.run_id = 17; event.kind = capture::Kind::State;
    event.id = event.state_id = 1; event.host_monotonic_ns = session.now_ns();
    event.position = {.125, -.25, .375, -.5, .625, -.75, .875};
    assert(session.offer(event) == capture::Offer::Accepted);
    assert(session.request_stop(17));
    assert(session.request_stop(17));
    session.join();
    const auto s = session.status();
    assert(s.joined && s.outcome_published && !s.runtime_failed);
    assert(s.capture.phase == capture::Phase::Complete);
    assert(fs::exists(root / "manual.capture.json"));
    assert(fs::exists(root / "manual/events.jsonl"));
    assert(fs::exists(root / "manual/session.log"));
    session.join();
  }
  {
    StreamingCaptureSession session({18, 1000000000ULL, 20000000ULL, 10, 8},
                                    root / "no-data", descriptor, io, 1ms, 500ms);
    assert(session.start());
    assert(until([&] { return session.status().capture.reason == capture::Reason::NoData; }));
    session.join();
    auto s = session.status();
    assert(s.capture.phase == capture::Phase::Failed);
    assert(s.outcome_published && !s.runtime_failed);
  }
  {
    StreamingCaptureSession session({24, 20000000ULL, 100000000ULL, 10, 8},
                                    root / "silent-after-first", descriptor, io,
                                    1ms, 500ms);
    assert(session.start());
    capture::Event e{};
    e.run_id = 24; e.kind = capture::Kind::State;
    e.id = e.state_id = 1; e.host_monotonic_ns = session.now_ns();
    e.position = {.125, -.25, .375, -.5, .625, -.75, .875};
    assert(session.offer(e) == capture::Offer::Accepted);
    assert(until([&] { return session.status().capture.reason == capture::Reason::Duration; }));
    session.join();
    const auto s = session.status();
    assert(s.outcome_published && !s.runtime_failed && s.capture.written == 1);
  }
  {
    StreamingCaptureSession session({25, 1000000000ULL, 100000000ULL, 10, 8},
                                    root / "destructor", descriptor, io, 1ms, 500ms);
    assert(session.start());
  }
  assert(!fs::exists(root / "destructor.capture.json"));
  struct BlockIO : FileOps {
    std::mutex mutex;
    std::condition_variable cv;
    std::atomic<bool> arm{false};
    std::string target = "signals.mcap.partial";
    bool blocked = false, release = false;
    ssize_t write(int fd, const void *data, size_t size) override {
      char link[512]{};
      const auto n = readlink(("/proc/self/fd/" + std::to_string(fd)).c_str(), link, 511);
      if (arm && n > 0 && std::string(link).find(target) != std::string::npos) {
        std::unique_lock<std::mutex> guard(mutex);
        blocked = true;
        cv.notify_all();
        cv.wait(guard, [this] { return release; });
      }
      return FileOps::write(fd, data, size);
    }
  } blocked_io;
  {
    StreamingCaptureSession session({19, 1000000000ULL, 500000000ULL, 10, 8},
                                    root / "blocked", descriptor, blocked_io, 1ms, 20ms);
    assert(session.start());
    assert(until([&] { return session.status().capture.phase == capture::Phase::Armed; }));
    blocked_io.arm = true;
    capture::Event e{};
    e.run_id = 19; e.kind = capture::Kind::State;
    e.id = e.state_id = 1; e.host_monotonic_ns = session.now_ns();
    e.position = {.125, -.25, .375, -.5, .625, -.75, .875};
    assert(session.offer(e) == capture::Offer::Accepted);
    {
      std::unique_lock<std::mutex> guard(blocked_io.mutex);
      assert(blocked_io.cv.wait_for(guard, 2s, [&] { return blocked_io.blocked; }));
    }
    assert(until([&] { return session.status().stalled; }));
    assert(session.request_stop(19));
    std::atomic<bool> joined{false};
    std::thread joiner([&] { session.join(); joined.store(true); });
    assert(!joined.load());
    {
      std::lock_guard<std::mutex> guard(blocked_io.mutex);
      blocked_io.release = true;
    }
    blocked_io.cv.notify_all();
    joiner.join();
    const auto s = session.status();
    assert(s.joined && s.stalled && !s.runtime_failed && s.outcome_published);
    assert(s.capture.phase == capture::Phase::Failed);
    assert(s.capture.reason == capture::Reason::WriterFailure);
    assert(fs::exists(root / "blocked.capture.json"));
  }
  BlockIO finish_io;
  finish_io.target = "quality.json.partial";
  {
    StreamingCaptureSession session({26, 1000000000ULL, 500000000ULL, 10, 8},
                                    root / "blocked-finish", descriptor, finish_io,
                                    1ms, 20ms);
    assert(session.start());
    finish_io.arm = true;
    capture::Event e{};
    e.run_id = 26; e.id = e.state_id = 1;
    e.host_monotonic_ns = session.now_ns();
    assert(session.offer(e) == capture::Offer::Accepted);
    assert(session.request_stop(26));
    {
      std::unique_lock<std::mutex> guard(finish_io.mutex);
      assert(finish_io.cv.wait_for(guard, 2s, [&] { return finish_io.blocked; }));
    }
    assert(until([&] { return session.status().stalled; }));
    std::atomic<bool> joined{false};
    std::thread joiner([&] { session.join(); joined.store(true); });
    assert(!joined.load());
    {
      std::lock_guard<std::mutex> guard(finish_io.mutex);
      finish_io.release = true;
    }
    finish_io.cv.notify_all();
    joiner.join();
    const auto s = session.status();
    assert(s.joined && s.stalled && !s.runtime_failed && s.outcome_published);
    assert(s.capture.phase == capture::Phase::Complete && s.capture.written == 1);
    assert(fs::exists(root / "blocked-finish.capture.json"));
    std::ifstream saved(root / "blocked-finish/session.log");
    const std::string log(std::istreambuf_iterator<char>{saved}, {});
    assert(log.find("stalled=1 runtime_failed=0") != std::string::npos);
  }
  BlockIO overflow_io;
  {
    StreamingCaptureSession session({28, 1000000000ULL, 500000000ULL, 10, 1},
                                    root / "overflow", descriptor, overflow_io,
                                    1ms, 2s);
    assert(session.start());
    overflow_io.arm = true;
    capture::Event e{};
    e.run_id = 28; e.kind = capture::Kind::State;
    e.id = e.state_id = 1; e.host_monotonic_ns = session.now_ns();
    e.position = {.125, -.25, .375, -.5, .625, -.75, .875};
    assert(session.offer(e) == capture::Offer::Accepted);
    {
      std::unique_lock<std::mutex> guard(overflow_io.mutex);
      assert(overflow_io.cv.wait_for(guard, 2s, [&] { return overflow_io.blocked; }));
    }
    e.id = e.state_id = 2; e.host_monotonic_ns = session.now_ns();
    assert(session.offer(e) == capture::Offer::Accepted);
    e.id = e.state_id = 3; e.host_monotonic_ns = session.now_ns();
    assert(session.offer(e) == capture::Offer::Rejected);
    {
      std::lock_guard<std::mutex> guard(overflow_io.mutex);
      overflow_io.release = true;
    }
    overflow_io.cv.notify_all();
    session.join();
    const auto s = session.status();
    assert(s.capture.reason == capture::Reason::Overflow);
    assert(s.capture.written == 2 && s.capture.rejected == 1);
    assert(s.outcome_published && !s.runtime_failed);
  }
  struct DiagnosticFailureIO : FileOps {
    ssize_t write(int fd, const void *data, size_t size) override {
      char link[512]{};
      const auto n = readlink(("/proc/self/fd/" + std::to_string(fd)).c_str(), link, 511);
      if (n > 0 && std::string(link).find("events.jsonl.partial") != std::string::npos) {
        errno = ENOSPC; return -1;
      }
      return FileOps::write(fd, data, size);
    }
  } diagnostics_io;
  {
    StreamingCaptureSession session({20, 1000000000ULL, 100000000ULL, 10, 8},
                                    root / "diagnostic-failure", descriptor, diagnostics_io,
                                    1ms, 500ms);
    assert(session.start());
    assert(session.request_stop(20));
    session.join();
    const auto s = session.status();
    assert(s.capture.phase == capture::Phase::Complete);
    assert(s.diagnostics_failed && s.runtime_failed && !s.outcome_published);
    assert(!fs::exists(root / "diagnostic-failure.capture.json"));
  }
  struct OutcomeFailureIO : FileOps {
    ssize_t write(int fd, const void *data, size_t size) override {
      char link[512]{};
      const auto n = readlink(("/proc/self/fd/" + std::to_string(fd)).c_str(), link, 511);
      if (n > 0 && std::string(link).find(".capture.json.partial") != std::string::npos) {
        errno = ENOSPC; return -1;
      }
      return FileOps::write(fd, data, size);
    }
  } outcome_io;
  {
    StreamingCaptureSession session({29, 1000000000ULL, 100000000ULL, 10, 8},
                                    root / "outcome-failure", descriptor, outcome_io,
                                    1ms, 500ms);
    assert(session.start());
    assert(session.request_stop(29));
    session.join();
    const auto s = session.status();
    assert(s.capture.phase == capture::Phase::Complete);
    assert(s.outcome_failed && s.runtime_failed && !s.diagnostics_failed);
    assert(!s.outcome_published);
    assert(!fs::exists(root / "outcome-failure.capture.json"));
  }
  for (const char *collision : {"signals.mcap", "quality.json"}) {
    const auto out = root / (std::string("session-collision-") + collision);
    StreamingCaptureSession session({30, 1000000000ULL, 100000000ULL, 10, 8},
                                    out, descriptor, io, 1ms, 500ms);
    {
      std::ofstream sentinel(out / collision);
      sentinel << "preserve";
    }
    assert(session.start());
    assert(session.request_stop(30));
    session.join();
    const auto s = session.status();
    assert(s.capture.phase == capture::Phase::Failed);
    assert(!s.capture.writer_in_call);
    std::ifstream saved(out / collision);
    const std::string bytes(std::istreambuf_iterator<char>{saved}, {});
    assert(bytes == "preserve");
    assert(fs::exists(out / (std::string(collision) + ".partial")));
    const fs::path sidecar(out.string() + ".capture.json");
    if (fs::exists(sidecar)) {
      std::ifstream f(sidecar);
      const std::string evidence(std::istreambuf_iterator<char>{f}, {});
      assert(evidence.find("\"phase\":\"Failed\"") != std::string::npos);
    }
  }
  {
    StreamingCaptureSession session({21, 1000000000ULL, 100000000ULL, 10, 8},
                                    root / "diagnostic-collision", descriptor, io,
                                    1ms, 500ms);
    assert(session.start());
    {
      std::ofstream sentinel(root / "diagnostic-collision/events.jsonl");
      sentinel << "preserve";
    }
    assert(session.request_stop(21));
    session.join();
    const auto s = session.status();
    assert(s.runtime_failed && s.diagnostics_failed && !s.outcome_published);
    std::ifstream saved(root / "diagnostic-collision/events.jsonl");
    std::string data(std::istreambuf_iterator<char>{saved}, {});
    assert(data == "preserve");
    assert(!fs::exists(root / "diagnostic-collision.capture.json"));
  }
  {
    StreamingCaptureSession session({27, 1000000000ULL, 100000000ULL, 10, 8},
                                    root / "ledger-limit", descriptor, io,
                                    1ms, 500ms, nullptr, 1);
    assert(session.start());
    assert(until([&] { return session.status().observed_events == 1; }));
    assert(session.request_stop(27));
    session.join();
    const auto s = session.status();
    assert(s.diagnostics_failed && s.runtime_failed && !s.outcome_published);
    assert(s.observed_events == 1);
    assert(!fs::exists(root / "ledger-limit.capture.json"));
  }
  struct FailSecondLaunch : SessionThreadLauncher {
    int calls = 0;
    std::thread launch(std::function<void()> task) override {
      if (++calls == 2) throw std::runtime_error("thread startup failure");
      return std::thread(std::move(task));
    }
  } launcher;
  {
    StreamingCaptureSession session({22, 1000000000ULL, 100000000ULL, 10, 8},
                                    root / "startup-failure", descriptor, io,
                                    1ms, 500ms, &launcher);
    assert(!session.start());
    const auto s = session.status();
    assert(s.joined && s.runtime_failed && s.coordinator_exited);
    assert(s.capture.reason == capture::Reason::Manual);
    assert(!fs::exists(root / "startup-failure.capture.json"));
  }
  if (!fixtures) fs::remove_all(root);
}
