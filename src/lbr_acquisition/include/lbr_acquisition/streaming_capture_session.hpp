#pragma once
#include "lbr_acquisition/capture_outcome.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace lbr_acquisition {
struct StreamingSessionStatus {
  capture::Status capture;
  bool started, writer_exited, coordinator_exited, stalled;
  bool diagnostics_failed, outcome_failed, runtime_failed, outcome_published, joined;
  size_t observed_events;
  capture::Reason requested_reason;
};
// Offline test seam for deterministic thread-construction failure.
struct SessionThreadLauncher {
  virtual ~SessionThreadLauncher() = default;
  virtual std::thread launch(std::function<void()> task) = 0;
};

// Offline fake-producer session. Exactly one external offer caller; its owner
// must cease/join that caller before join/destruction. FileOps must outlive us.
class StreamingCaptureSession {
public:
  StreamingCaptureSession(capture::Config config, std::filesystem::path directory,
                          std::filesystem::path descriptor, FileOps &io,
                          std::chrono::nanoseconds poll_period,
                          std::chrono::nanoseconds writer_stall_budget,
                          SessionThreadLauncher *launcher = nullptr,
                          size_t diagnostic_capacity = 32);
  ~StreamingCaptureSession();
  StreamingCaptureSession(const StreamingCaptureSession &) = delete;
  StreamingCaptureSession &operator=(const StreamingCaptureSession &) = delete;
  bool start();
  capture::Offer offer(const capture::Event &event, bool eligible = true) noexcept;
  bool request_stop(uint64_t run_id,
                    capture::Reason reason = capture::Reason::Manual) noexcept;
  StreamingSessionStatus status() const noexcept;
  void join();
  static uint64_t now_ns() noexcept;
private:
  struct Entry {
    uint64_t at_ns;
    capture::Phase phase;
    capture::Reason reason;
    uint32_t flags;
  };
  static constexpr size_t ledger_limit = 32;
  void coordinator_loop() noexcept;
  void writer_loop() noexcept;
  void observe(const capture::Status &, uint64_t) noexcept;
  void join_workers(bool publish) noexcept;
  void publish_diagnostics();
  const capture::Config config_;
  const std::filesystem::path directory_;
  FileOps &io_;
  const uint64_t poll_ns_, stall_ns_;
  const size_t ledger_capacity_;
  SessionThreadLauncher *launcher_;
  capture::Recorder recorder_;
  StreamingCaptureWriter sink_;
  std::thread coordinator_, writer_;
  mutable std::mutex owner_mutex_, start_mutex_;
  std::condition_variable start_cv_;
  std::atomic<bool> start_called_{false}, core_started_{false};
  // None -> first requested reason is one indivisible, first-wins transition.
  std::atomic<capture::Reason> requested_reason_{capture::Reason::None};
  std::atomic<bool> writer_exited_{false};
  std::atomic<bool> coordinator_exited_{false}, stalled_{false};
  std::atomic<bool> diagnostics_failed_{false}, outcome_failed_{false};
  std::atomic<bool> runtime_failed_{false};
  std::atomic<bool> outcome_published_{false}, joined_{false};
  std::atomic<size_t> observed_events_{0};
  std::atomic<uint64_t> call_sequence_{0}, call_started_ns_{0};
  std::atomic<bool> call_active_{false};
  Entry ledger_[ledger_limit]{};
  size_t ledger_count_ = 0;
};
} // namespace lbr_acquisition
