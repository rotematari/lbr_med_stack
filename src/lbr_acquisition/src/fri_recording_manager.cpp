// PHD-FRI-RECORDER: bounded owner-thread lifecycle and cycle acknowledgement.
#include "lbr_acquisition/fri_recording_manager.hpp"
#include <cassert>
#include <limits>

namespace lbr_acquisition {
namespace c = capture;
FriRecordingManager::FriRecordingManager()
    : route_(std::make_shared<detail::RecordingRoute>()),
      adapter_(new RecorderAdapter(route_)) {}
FriRecordingManager::~FriRecordingManager() {
  producer_stopped(); // Caller has established the documented quiescence precondition.
  if (recording_) recording_->wait();
}
uint64_t FriRecordingManager::publish(RecorderAdapter *target) noexcept {
  assert(route_->requested.load() == route_->acknowledged.load(std::memory_order_acquire));
  route_->target = target;
  route_->requested.store(++sequence_, std::memory_order_release);
  return sequence_;
}
bool FriRecordingManager::prepare(c::Config config, const std::filesystem::path &directory,
                                   const std::filesystem::path &descriptor,
                                   RecorderAdapter::Clock clock) {
  if (recording_ || failed_ || quiesced_ || route_->source_failed.load() ||
      !config.run_id || config.run_id <= last_id_ ||
      sequence_ > std::numeric_limits<uint64_t>::max() - 2) return false;
  last_id_ = config.run_id;
  last_ = {};
  last_.run_id = config.run_id;
  attach_ = detach_ = 0;
  started_ = stopping_ = false;
  try { recording_ = FriRecording::prepare(config, directory, descriptor, clock); }
  catch (...) {
    failed_ = true;
    last_.phase = RecordingManagerPhase::Failed;
    last_.retired = true;
    return false;
  }
  last_.phase = RecordingManagerPhase::Prepared;
  return true;
}
bool FriRecordingManager::start(uint64_t id) {
  if (!recording_ || id != last_.run_id || started_ || stopping_ || failed_ ||
      quiesced_ || route_->source_failed.load()) return false;
  if (!recording_->start()) {
    stopping_ = failed_ = true;
    return false;
  }
  started_ = true;
  attach_ = publish(recording_->adapter().get());
  return true;
}
bool FriRecordingManager::request_stop(uint64_t id) noexcept {
  if (!id || id != last_.run_id) return false;
  if (!recording_) return last_.retired;
  // Source loss is recorded through the run's first-wins stop; finish() classifies
  // only from the session outcome. prepare() already refuses runs after source loss.
  if (route_->source_failed.load()) recording_->source_stopped();
  else recording_->request_stop(id);
  stopping_ = true;
  poll();
  return true;
}
RecordingManagerStatus FriRecordingManager::status() const noexcept {
  auto result = last_;
  result.source_failed = route_->source_failed.load();
  result.owns_run = bool(recording_);
  if (!recording_) return result;
  const auto ack = route_->acknowledged.load(std::memory_order_acquire);
  result.detached = quiesced_ || !attach_ || (detach_ && ack >= detach_);
  result.attached = attach_ && ack >= attach_ && !result.detached;
  result.session = recording_->status();
  result.phase = stopping_ ? RecordingManagerPhase::Stopping : !started_
      ? RecordingManagerPhase::Prepared : result.session.capture.has_first
      ? RecordingManagerPhase::Recording : RecordingManagerPhase::Armed;
  return result;
}
RecordingManagerStatus FriRecordingManager::poll() noexcept {
  if (!recording_) return status();
  if (route_->source_failed.load()) {
    recording_->source_stopped();
    stopping_ = true;
  }
  const auto s = recording_->status();
  if (started_ && (s.capture.reason != c::Reason::None || s.runtime_failed ||
                  s.requested_reason != c::Reason::None)) stopping_ = true;
  // Do not replace a pending attach. Its acknowledgement proves the FRI thread
  // has consumed that request; only then publish detach in the same mailbox.
  if (stopping_ && !quiesced_ && attach_ && !detach_ &&
      route_->acknowledged.load(std::memory_order_acquire) == attach_)
    detach_ = publish(nullptr);
  return status();
}
bool FriRecordingManager::finish(uint64_t id) {
  if (!id || id != last_.run_id) return false;
  if (!recording_) return last_.retired;
  auto s = poll();
  if (!stopping_ || !s.detached || (started_ && !s.session.joined)) return false;
  recording_->wait();
  s.session = recording_->status();
  const auto &r = s.session;
  const bool complete = r.started && r.capture.has_first && r.capture.phase == c::Phase::Complete &&
      !r.capture.failure_flags && r.capture.written == r.capture.enqueued &&
      !r.capture.abandoned && !r.capture.write_uncertain && r.joined && r.outcome_published &&
      !r.runtime_failed && !r.diagnostics_failed && !r.outcome_failed;
  s.phase = !started_ && !failed_ ? RecordingManagerPhase::Cancelled : complete && !failed_
      ? RecordingManagerPhase::Finalized : RecordingManagerPhase::Failed;
  if (s.phase == RecordingManagerPhase::Failed) failed_ = true;
  s.retired = true;
  s.owns_run = s.attached = false;
  s.detached = true;
  recording_.reset(); // Owner only; callback has acknowledged detach or terminated.
  last_ = s;
  return true;
}
void FriRecordingManager::producer_stopped() noexcept {
  quiesced_ = true;
  route_->closed.store(true);
  route_->source_failed.store(true);
  if (recording_) {
    recording_->source_stopped(); // No effect if an earlier stop cause was latched.
    stopping_ = true;
  }
}
} // namespace lbr_acquisition
