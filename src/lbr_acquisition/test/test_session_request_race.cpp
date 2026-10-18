#include "lbr_acquisition/streaming_capture_session.hpp"
#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <mutex>
#include <thread>
#include <unistd.h>
using namespace lbr_acquisition;
using namespace std::chrono_literals;
namespace fs = std::filesystem;
namespace {
std::atomic<int> calls{0};
std::mutex mutex;
std::condition_variable cv;
bool paused = false, release = false;
}
namespace lbr_acquisition {
void session_test_before_stop_publish() noexcept {
  if (calls.fetch_add(1) != 0) return;
  std::unique_lock<std::mutex> guard(mutex);
  paused = true;
  cv.notify_all();
  cv.wait(guard, [] { return release; });
}
}
int main(int argc, char **argv) {
  assert(argc == 2);
  char pattern[] = "/tmp/acq-stop-race-XXXXXX";
  const fs::path root(mkdtemp(pattern));
  assert(!root.empty());
  FileOps io;
  {
    StreamingCaptureSession session({31, 1000000000ULL, 100000000ULL, 10, 8},
                                    root / "run", argv[1], io, 1ms, 1s);
    std::thread first([&] {
      assert(session.request_stop(31, capture::Reason::Duration));
    });
    {
      std::unique_lock<std::mutex> guard(mutex);
      assert(cv.wait_for(guard, 2s, [] { return paused; }));
    }
    assert(!session.request_stop(999, capture::Reason::Overflow));
    assert(session.request_stop(31, capture::Reason::Manual));
    {
      std::lock_guard<std::mutex> guard(mutex);
      release = true;
    }
    cv.notify_all();
    first.join();
    assert(session.status().requested_reason == capture::Reason::Manual);
    assert(!session.start());
  }
  fs::remove_all(root);
}
