#pragma once
#include "lbr_acquisition/records.hpp"
#include <atomic>
#include <boost/lockfree/spsc_queue.hpp>
#include <cstddef>

namespace lbr_acquisition::capture {
struct Config {
  uint64_t run_id, duration_ns, first_sample_timeout_ns, max_events;
  size_t queue_capacity;
};
enum class Offer { Accepted, Inactive, Stale, Ineligible, Rejected };
enum class Reason : uint32_t {
  None,
  Manual,
  Duration,
  NoData,
  Overflow,
  InvalidEvent,
  EventLimit,
  SourceUnavailable,
  ClockRegression,
  WriterFailure
};
enum class Phase {
  Ready,
  Armed,
  Recording,
  Stopping,
  Finalizing,
  Complete,
  Failed
};
struct Status {
  Phase phase;
  Reason reason;
  // For each material Reason r: bit (1u << static_cast<uint32_t>(r)).
  // None, Manual and Duration never set a bit. Multiple faults are retained.
  uint32_t failure_flags;
  bool has_first, writer_in_call, writer_done;
  uint64_t first_state_ns, last_state_ns;
  uint64_t attempted, enqueued, rejected, written, abandoned, write_uncertain;
};
struct Writer {
  virtual ~Writer() = default;
  virtual void write(const Event &) = 0;
  virtual void finish() = 0;
};
// Single-use endpoint. Exactly one offer caller, one writer_step caller, and a
// serialized coordinator (start/stop/tick). status may be read concurrently.
// External owners must prevent future calls and join all roles before
// destruction. No owned threads, timer, files or SDK. Complete means sink
// acknowledged finish.
class Recorder {
public:
  explicit Recorder(Config);
  Recorder(const Recorder &) = delete;
  Recorder &operator=(const Recorder &) = delete;
  bool start(uint64_t run_id, uint64_t now_ns) noexcept;
  bool stop(uint64_t run_id, Reason = Reason::Manual) noexcept;
  void tick(uint64_t now_ns) noexcept;
  Offer offer(const Event &, bool eligible = true) noexcept;
  bool writer_step(Writer &) noexcept;
  Status status() const noexcept;

private:
  static Config validate(Config);
  void close(Reason) noexcept;
  static bool material(Reason) noexcept;
  const Config config_;
  boost::lockfree::spsc_queue<Event> queue_;
  static constexpr uint32_t closed = 1, active = 2;
  std::atomic<uint32_t> gate_{closed}, failures_{0};
  std::atomic<Reason> reason_{Reason::None};
  std::atomic<bool> started_{false}, has_first_{false};
  std::atomic<bool> writer_in_call_{false}, writer_done_{false};
  std::atomic<uint64_t> arm_ns_{0}, first_ns_{0}, last_state_ns_{0};
  std::atomic<uint64_t> attempted_{0}, enqueued_{0}, rejected_{0};
  std::atomic<uint64_t> written_{0}, abandoned_{0}, uncertain_{0};
  // Producer-only and writer-only state respectively; never read by status.
  uint64_t last_state_id_ = 0, last_submission_id_ = 0, last_event_ns_ = 0;
  bool write_failed_ = false;
};
} // namespace lbr_acquisition::capture
