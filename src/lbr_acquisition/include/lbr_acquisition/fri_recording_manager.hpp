// PHD-FRI-RECORDER: reusable owner of single-use recordings; no robot commands.
#pragma once
#include "lbr_acquisition/fri_adapter.hpp"
#include "lbr_acquisition/streaming_capture_session.hpp"
#include <atomic>

namespace lbr_acquisition {
namespace detail {
// One owner publishes only after acknowledgement of the previous request.
// target is immutable from request publication through acknowledgement. The
// owner retains its pointee until a subsequent detach is acknowledged.
struct RecordingRoute {
  RecorderAdapter *target = nullptr;
  std::atomic<uint64_t> requested{0}, acknowledged{0};
  std::atomic<bool> source_failed{false}, closed{false};
};
static_assert(std::atomic<uint64_t>::is_always_lock_free &&
              std::atomic<bool>::is_always_lock_free,
              "FRI recording routing requires lock-free atomics");
} // namespace detail

enum class RecordingManagerPhase { Idle, Prepared, Armed, Recording, Stopping,
                                   Finalized, Cancelled, Failed };
struct RecordingManagerStatus {
  uint64_t run_id = 0;
  RecordingManagerPhase phase = RecordingManagerPhase::Idle;
  bool attached = false, detached = true, retired = false, owns_run = false;
  bool source_failed = false;
  StreamingSessionStatus session{};
};

// All manager methods are serialized on ONE non-FRI owner thread. Poll regularly
// to progress automatic stops and handoffs; there is no manager worker thread.
// The stable adapter alone is used by the FRI producer. Its source_stopped()
// notification may also be called from another thread (atomic notification only).
class FriRecordingManager {
public:
  FriRecordingManager();
  // PRECONDITION: producer is joined/quiescent and will never call again.
  ~FriRecordingManager();
  FriRecordingManager(const FriRecordingManager &) = delete;
  FriRecordingManager &operator=(const FriRecordingManager &) = delete;
  std::shared_ptr<RecorderAdapter> adapter() const noexcept { return adapter_; }
  // New absolute directory and strictly increasing nonzero ID. Busy/stale IDs
  // return false without changing the run. Preparation errors latch Failed;
  // partial files stay on disk. The attempted ID is consumed even on error.
  bool prepare(capture::Config, const std::filesystem::path &directory,
               const std::filesystem::path &descriptor, RecorderAdapter::Clock = nullptr);
  bool start(uint64_t run_id);
  // Recognition only, not saved data or detached callbacks. No waits or I/O.
  bool request_stop(uint64_t run_id) noexcept;
  RecordingManagerStatus poll() noexcept;
  RecordingManagerStatus status() const noexcept;
  // False until stopping, detached and session workers joined. Then joins the
  // finalizer and releases the run on the owner thread (may block). True means
  // retired; inspect phase/session separately for failure and data qualification.
  bool finish(uint64_t run_id);
  // Terminal notification ONLY AFTER producer termination was actually confirmed.
  // Neither socket closure nor a timeout establishes this precondition.
  void producer_stopped() noexcept;
private:
  uint64_t publish(RecorderAdapter *) noexcept;
  std::shared_ptr<detail::RecordingRoute> route_;
  std::shared_ptr<RecorderAdapter> adapter_;
  std::shared_ptr<FriRecording> recording_;
  RecordingManagerStatus last_{};
  uint64_t last_id_ = 0, sequence_ = 0, attach_ = 0, detach_ = 0;
  bool started_ = false, stopping_ = false, failed_ = false, quiesced_ = false;
};
} // namespace lbr_acquisition
