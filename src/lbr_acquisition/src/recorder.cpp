#include "lbr_acquisition/recorder.hpp"
#include <cmath>
#include <stdexcept>

namespace lbr_acquisition::capture {
#ifdef LBR_CAPTURE_TEST_HOOKS
// Defined only by the separate deterministic test executable. No calls or
// symbol are present in acquisition_capture, the production library.
void test_hook(unsigned) noexcept;
#define CAPTURE_POINT(n) test_hook(n)
#else
#define CAPTURE_POINT(n) ((void)0)
#endif
namespace {
struct ActiveScope {
  std::atomic<uint32_t> &gate;
  ~ActiveScope() { gate.fetch_and(~uint32_t{2}); }
};
} // namespace
Config Recorder::validate(Config c) {
  if (!c.run_id || !c.duration_ns || !c.first_sample_timeout_ns ||
      !c.max_events || c.max_events > 10000 || !c.queue_capacity ||
      c.queue_capacity > 10000)
    throw std::invalid_argument("invalid offline recorder limits");
  return c;
}
Recorder::Recorder(Config c)
    : config_(validate(c)), queue_(config_.queue_capacity) {
  // Check every atomic value type actually used by this endpoint, plus Boost's
  // queue atomics. These checks are prerequisites, not a deadline guarantee.
  if (!queue_.is_lock_free() || !gate_.is_lock_free() ||
      !reason_.is_lock_free() || !started_.is_lock_free() ||
      !attempted_.is_lock_free())
    throw std::runtime_error(
        "capture requires lock-free atomics on this target");
}
bool Recorder::material(Reason r) noexcept {
  return r != Reason::None && r != Reason::Manual && r != Reason::Duration;
}
void Recorder::close(Reason r) noexcept {
  if (material(r))
    failures_.fetch_or(uint32_t{1} << static_cast<uint32_t>(r));
  auto expected = Reason::None;
  reason_.compare_exchange_strong(expected, r);
  gate_.fetch_or(closed);
}
bool Recorder::start(uint64_t run_id, uint64_t now_ns) noexcept {
  if (run_id != config_.run_id || reason_.load() != Reason::None)
    return false;
  if (started_.load())
    return true; // Never re-open an existing run.
  arm_ns_.store(now_ns);
  started_.store(true);
  // Coordinator serializes start/stop/tick. Before this first open the writer
  // cannot have an event or finalize without an earlier stop (checked above).
  gate_.fetch_and(~closed);
  return true;
}
bool Recorder::stop(uint64_t run_id, Reason reason) noexcept {
  if (run_id != config_.run_id || reason == Reason::None ||
      static_cast<uint32_t>(reason) >
          static_cast<uint32_t>(Reason::WriterFailure))
    return false;
  if (reason_.load() == Reason::None)
    close(reason);
  return true;
}
void Recorder::tick(uint64_t now_ns) noexcept {
  if (!started_.load() || reason_.load() != Reason::None)
    return;
  if (now_ns < arm_ns_.load()) {
    close(Reason::ClockRegression);
    return;
  }
  if (!has_first_.load()) {
    if (now_ns - arm_ns_.load() >= config_.first_sample_timeout_ns)
      close(Reason::NoData);
  } else {
    const auto first = first_ns_.load();
    // now may have been sampled just before a concurrent first publication.
    if (now_ns >= first && now_ns - first >= config_.duration_ns)
      close(Reason::Duration);
  }
}
Offer Recorder::offer(const Event &e, bool eligible) noexcept {
  if (e.run_id != config_.run_id)
    return Offer::Stale;
  uint32_t expected = 0;
  const bool admitted = gate_.compare_exchange_strong(expected, active);
  CAPTURE_POINT(1);
  if (!admitted)
    return Offer::Inactive;
  ActiveScope scope{gate_};
  if (!eligible) {
    if (has_first_.load())
      close(Reason::SourceUnavailable);
    return Offer::Ineligible;
  }
  const auto arm = arm_ns_.load();
  const bool first_present = has_first_.load();
  if (e.host_monotonic_ns < arm ||
      (first_present && e.host_monotonic_ns < last_event_ns_)) {
    attempted_.fetch_add(1);
    rejected_.fetch_add(1);
    close(Reason::ClockRegression);
    return Offer::Rejected;
  }
  if (!first_present &&
      e.host_monotonic_ns - arm >= config_.first_sample_timeout_ns) {
    close(Reason::NoData);
    return Offer::Inactive;
  }
  if (first_present &&
      e.host_monotonic_ns - first_ns_.load() >= config_.duration_ns) {
    close(Reason::Duration);
    return Offer::Inactive;
  }
  attempted_.fetch_add(1);
  const bool valid_state = e.kind == Kind::State && e.id &&
                           e.id > last_state_id_ && e.state_id == e.id;
  const bool valid_submission = e.kind == Kind::Submission && e.id &&
                                e.id > last_submission_id_ && last_state_id_ &&
                                e.state_id == last_state_id_;
  uint32_t nonfinite_mask = 0;
  for (size_t i = 0; i < e.position.size(); ++i)
    if (!std::isfinite(e.position[i])) nonfinite_mask |= uint32_t{1} << i;
  constexpr uint32_t known_metadata = ControllerTime | SamplePeriod | Session |
                                      Quality | Control | Command;
  const bool valid_payload = nonfinite_mask == e.nonfinite_joint_mask &&
      !(e.metadata.presence & ~known_metadata) &&
      (e.kind != Kind::Submission || !e.metadata.presence);
  if ((!valid_state && !valid_submission) || !valid_payload) {
    rejected_.fetch_add(1);
    close(Reason::InvalidEvent);
    return Offer::Rejected;
  }
  if (enqueued_.load() >= config_.max_events) {
    rejected_.fetch_add(1);
    close(Reason::EventLimit);
    return Offer::Rejected;
  }
  CAPTURE_POINT(2);
  if (!queue_.push(e)) {
    rejected_.fetch_add(1);
    close(Reason::Overflow);
    return Offer::Rejected;
  }
  enqueued_.fetch_add(1);
  last_event_ns_ = e.host_monotonic_ns;
  if (e.kind == Kind::State) {
    last_state_id_ = e.id;
    last_state_ns_.store(e.host_monotonic_ns);
    if (!first_present) {
      first_ns_.store(e.host_monotonic_ns);
      has_first_.store(true);
    }
  } else {
    last_submission_id_ = e.id;
  }
  CAPTURE_POINT(3);
  return Offer::Accepted;
}
bool Recorder::writer_step(Writer &writer) noexcept {
  if (writer_done_.load())
    return false;
  // Observe producer release BEFORE the final empty pop: a pop performed before
  // this acquire cannot establish that an admitted publication is finished.
  const bool quiescent =
      reason_.load() != Reason::None && gate_.load() == closed;
  Event event;
  if (queue_.pop(event)) {
    if (write_failed_) {
      abandoned_.fetch_add(1);
    } else {
      writer_in_call_.store(true);
      try {
        writer.write(event);
        written_.fetch_add(1);
      } catch (...) {
        uncertain_.fetch_add(1);
        write_failed_ = true;
        close(Reason::WriterFailure);
      }
      writer_in_call_.store(false);
    }
    return true;
  }
  if (!quiescent)
    return false;
  if (!write_failed_) {
    writer_in_call_.store(true);
    try {
      writer.finish();
    } catch (...) {
      close(Reason::WriterFailure);
    }
    writer_in_call_.store(false);
  }
  writer_done_.store(true);
  return true;
}
Status Recorder::status() const noexcept {
  Status s{};
  // Observe terminal publication first: a true value orders every fault/counter
  // update before the following reads. False permits a conservative live view.
  s.writer_done = writer_done_.load();
  s.reason = reason_.load();
  s.failure_flags = failures_.load();
  CAPTURE_POINT(4);
  s.has_first = has_first_.load();
  s.writer_in_call = writer_in_call_.load();
  s.first_state_ns = first_ns_.load();
  s.last_state_ns = last_state_ns_.load();
  s.attempted = attempted_.load();
  s.enqueued = enqueued_.load();
  s.rejected = rejected_.load();
  s.written = written_.load();
  s.abandoned = abandoned_.load();
  s.write_uncertain = uncertain_.load();
  if (s.writer_done)
    s.phase = s.failure_flags ? Phase::Failed : Phase::Complete;
  else if (s.reason != Reason::None)
    s.phase = gate_.load() & active ? Phase::Stopping : Phase::Finalizing;
  else
    s.phase = !started_.load() ? Phase::Ready
              : s.has_first    ? Phase::Recording
                               : Phase::Armed;
  return s;
}
#undef CAPTURE_POINT
} // namespace lbr_acquisition::capture
