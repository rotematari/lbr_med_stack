#include "lbr_acquisition/recorder.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
using namespace lbr_acquisition::capture;
struct Sink : Writer {
  std::vector<Event> events;
  bool fail_write = false, fail_finish = false;
  unsigned finishes = 0;
  void write(const Event &e) override {
    events.push_back(e); // Failure can occur AFTER the sink accepted the event.
    if (fail_write)
      throw std::runtime_error("ambiguous write");
  }
  void finish() override {
    ++finishes;
    if (fail_finish)
      throw std::runtime_error("finish failed");
  }
};
Event state(uint64_t id, uint64_t time) {
  Event e{};
  e.kind = Kind::State;
  e.run_id = 7;
  e.id = id;
  e.state_id = id;
  e.host_monotonic_ns = time;
  e.position = {.125, -0., -.25, .375, -.5, .625, -.75};
  return e;
}
Event submission(uint64_t id, uint64_t parent, uint64_t time) {
  auto e = state(id, time);
  e.kind = Kind::Submission;
  e.state_id = parent;
  e.position = {.5, -.125, .25, -.375, .5, -.625, .75};
  return e;
}
Config config(size_t capacity = 2) { return {7, 20, 50, 10000, capacity}; }
void drain(Recorder &r, Sink &sink) {
  for (unsigned i = 0; i < 10002 && !r.status().writer_done; ++i)
    r.writer_step(sink);
  assert(r.status().writer_done);
  auto s = r.status();
  assert(s.enqueued == s.written + s.abandoned + s.write_uncertain);
  assert(s.attempted == s.enqueued + s.rejected);
}
void owns_records_and_reports_overflow() {
  Recorder r(config());
  Sink sink;
  assert(!r.writer_step(sink));
  assert(sink.finishes == 0);
  assert(r.offer(state(1, 100)) == Offer::Inactive);
  assert(!r.start(8, 100));
  assert(r.start(7, 100));
  auto e = state(1, 110);
  e.metadata.presence = 63;
  e.metadata.controller_sec = 123;
  e.torque_presence = MeasuredTorque | CommandedTorque | EstimatedExternalTorque;
  e.tau_measured_nm = {1.25, -0., -2.5, 3.75, -5., 6.25, -7.5};
  e.tau_commanded_nm.fill(2.5);
  e.tau_external_estimated_nm.fill(-.25);
  e.metadata.presence |= Safety | Tracking;
  e.metadata.safety_state = 1;
  e.metadata.tracking_performance = .5;
  e.signal_presence = CommandedJointPosition;
  e.q_commanded_rad = {-.5, -0., .25, -.375, .5, -.625, .75};
  assert(r.offer(e) == Offer::Accepted);
  e.q_commanded_rad.fill(99);
  e.position.fill(99);
  e.tau_measured_nm.fill(99);
  e.tau_commanded_nm.fill(99);
  e.tau_external_estimated_nm.fill(99);
  e.metadata.controller_sec = 999;
  assert(r.offer(submission(1, 1, 111)) == Offer::Accepted);
  assert(r.offer(state(2, 112)) == Offer::Rejected);
  assert(r.offer(state(3, 113)) == Offer::Inactive);
  drain(r, sink);
  auto s = r.status();
  assert(s.phase == Phase::Failed && s.reason == Reason::Overflow);
  assert(s.attempted == 3 && s.rejected == 1 && s.written == 2);
  assert(sink.events.size() == 2 && sink.events[0].position[0] == .125);
  assert(std::signbit(sink.events[0].position[1]));
  assert(sink.events[0].metadata.controller_sec == 123);
  assert(sink.events[0].torque_presence == 7);
  assert(sink.events[0].tau_measured_nm[0] == 1.25 &&
         std::signbit(sink.events[0].tau_measured_nm[1]));
  assert(sink.events[0].tau_commanded_nm[6] == 2.5);
  assert(sink.events[0].tau_external_estimated_nm[6] == -.25);
  assert(sink.events[0].metadata.presence == 255);
  assert(sink.events[0].metadata.safety_state == 1 &&
         sink.events[0].metadata.tracking_performance == .5);
  assert(sink.events[0].signal_presence == CommandedJointPosition);
  assert(sink.events[0].q_commanded_rad[0] == -.5 &&
         std::signbit(sink.events[0].q_commanded_rad[1]));
  assert(sink.events[1].position[0] == .5 && sink.events[1].state_id == 1);
  assert(sink.finishes == 1 && !r.writer_step(sink));
}
void duration_is_first_state_and_duplicate_start_does_not_extend_it() {
  Recorder r(config(4));
  Sink sink;
  r.start(7, 100);
  assert(r.start(7, 140)); // preserves initial arm time
  assert(r.offer(state(1, 149), false) == Offer::Ineligible);
  assert(r.offer(state(1, 149)) == Offer::Accepted);
  assert(r.offer(state(2, 168)) == Offer::Accepted);
  assert(r.offer(state(3, 169)) == Offer::Inactive);
  drain(r, sink);
  auto s = r.status();
  assert(s.phase == Phase::Complete && s.reason == Reason::Duration);
  assert(s.has_first && s.first_state_ns == 149 && s.last_state_ns == 168);
  assert(s.enqueued == 2 && s.rejected == 0);
  assert(!r.start(7, 200));
}
void silent_expiry_and_cancellation() {
  {
    Recorder r(config());
    Sink sink;
    r.start(7, 100);
    r.tick(149);
    assert(r.status().phase == Phase::Armed);
    r.tick(150);
    drain(r, sink);
    assert(r.status().reason == Reason::NoData &&
           r.status().phase == Phase::Failed);
    assert(r.offer(state(1, 151)) == Offer::Inactive);
  }
  {
    Recorder r(config());
    Sink sink;
    assert(!r.stop(8));
    assert(r.stop(7));
    assert(r.stop(7));
    assert(!r.start(7, 100));
    drain(r, sink);
    assert(r.status().reason == Reason::Manual && r.status().enqueued == 0);
  }
  {
    Recorder r(config());
    Sink sink;
    r.start(7, 100);
    r.offer(state(1, 110));
    r.tick(130);
    drain(r, sink);
    assert(r.status().reason == Reason::Duration);
  }
  {
    Recorder r(config());
    Sink sink;
    r.start(7, 100);
    assert(r.offer(state(1, 150)) == Offer::Inactive);
    drain(r, sink);
    assert(r.status().reason == Reason::NoData);
  }
}
void bad_associations_are_not_written() {
  for (int scenario = 0; scenario < 4; ++scenario) {
    Recorder r(config(4));
    Sink sink;
    r.start(7, 100);
    if (scenario)
      assert(r.offer(state(1, 101)) == Offer::Accepted);
    Event e = scenario == 0   ? submission(1, 1, 102)
              : scenario == 1 ? submission(1, 2, 102)
              : scenario == 2 ? state(1, 102)
                              : state(2, 102);
    if (scenario == 3)
      e.kind = static_cast<Kind>(99);
    assert(r.offer(e) == Offer::Rejected);
    drain(r, sink);
    assert(r.status().reason == Reason::InvalidEvent &&
           r.status().rejected == 1);
  }
}
void failure_accounting_and_finish() {
  {
    Recorder r(config(4));
    Sink sink;
    r.start(7, 100);
    r.offer(state(1, 101));
    r.offer(state(2, 102));
    sink.fail_write = true;
    assert(r.writer_step(sink));
    drain(r, sink);
    auto s = r.status();
    assert(s.phase == Phase::Failed && s.write_uncertain == 1 &&
           s.abandoned == 1);
    assert(s.written == 0 && sink.events.size() == 1 && sink.finishes == 0);
  }
  {
    Recorder r(config());
    Sink sink;
    r.start(7, 100);
    r.offer(state(1, 101));
    r.stop(7);
    sink.fail_finish = true;
    drain(r, sink);
    auto s = r.status();
    assert(s.phase == Phase::Failed && s.reason == Reason::Manual);
    assert(s.failure_flags && s.written == 1 && sink.finishes == 1);
  }
}
void wraparound_limits_and_raw_anomalies() {
  Config c = config(2);
  c.duration_ns = 100000;
  Recorder r(c);
  Sink sink;
  r.start(7, 0);
  for (uint64_t i = 1; i <= 200; ++i) {
    auto e = state(i, i);
    e.position[0] = std::numeric_limits<double>::quiet_NaN();
    e.nonfinite_joint_mask = 1;
    e.metadata.presence = 1;
    e.metadata.controller_nsec = UINT32_MAX;
    assert(r.offer(e) == Offer::Accepted);
    assert(r.writer_step(sink));
  }
  r.stop(7);
  drain(r, sink);
  assert(sink.events.size() == 200);
  for (size_t i = 0; i < 200; ++i) {
    assert(sink.events[i].id == i + 1);
    assert(std::isnan(sink.events[i].position[0]));
    assert(sink.events[i].metadata.controller_nsec == UINT32_MAX);
  }
  c.max_events = 1;
  Recorder limited(c);
  Sink ls;
  limited.start(7, 0);
  assert(limited.offer(state(1, 1)) == Offer::Accepted);
  assert(limited.offer(state(2, 2)) == Offer::Rejected);
  drain(limited, ls);
  assert(limited.status().reason == Reason::EventLimit);
}
void stale_clock_and_source_policy() {
  Recorder r(config(4));
  Sink sink;
  r.start(7, 100);
  auto stale = state(1, 101);
  stale.run_id = 8;
  assert(r.offer(stale) == Offer::Stale && r.status().attempted == 0);
  r.offer(state(1, 110));
  assert(r.offer(state(2, 109)) == Offer::Rejected);
  drain(r, sink);
  assert(r.status().reason == Reason::ClockRegression);
  Recorder lost(config());
  Sink ls;
  lost.start(7, 100);
  lost.offer(state(1, 101));
  assert(lost.offer(state(2, 102), false) == Offer::Ineligible);
  drain(lost, ls);
  assert(lost.status().reason == Reason::SourceUnavailable);
  Config c = config();
  c.first_sample_timeout_ns = 100;
  Recorder nearmax(c);
  Sink ms;
  nearmax.start(7, UINT64_MAX - 10);
  nearmax.offer(state(1, UINT64_MAX - 5));
  nearmax.tick(UINT64_MAX);
  assert(nearmax.status().phase == Phase::Recording);
  nearmax.stop(7);
  drain(nearmax, ms);
}
void earlier_tick_snapshot_does_not_fault_a_new_first_state() {
  Recorder r(config());
  Sink sink;
  r.start(7, 100);
  // Coordinator sampled 109, then producer published the first State at 110.
  // A late invocation with that snapshot is not a regressed producer clock.
  assert(r.offer(state(1, 110)) == Offer::Accepted);
  r.tick(109);
  assert(r.status().phase == Phase::Recording);
  r.tick(130);
  drain(r, sink);
  assert(r.status().reason == Reason::Duration);
}
void invalid_configuration() {
  for (int i = 0; i < 6; ++i) {
    Config c = config();
    if (i == 0)
      c.run_id = 0;
    if (i == 1)
      c.duration_ns = 0;
    if (i == 2)
      c.first_sample_timeout_ns = 0;
    if (i == 3)
      c.max_events = 0;
    if (i == 4)
      c.queue_capacity = 0;
    if (i == 5)
      c.queue_capacity = 10001;
    bool caught = false;
    try {
      Recorder r(c);
    } catch (const std::invalid_argument &) {
      caught = true;
    }
    assert(caught);
  }
}
void malformed_payload_is_rejected_before_the_writer() {
  for (unsigned fault = 0; fault < 8; ++fault) {
    Recorder r(config());
    Sink sink;
    assert(r.start(7, 100));
    auto e = state(1, 101);
    if (fault == 0) e.nonfinite_joint_mask = 1;
    if (fault == 1) e.position[0] = std::numeric_limits<double>::quiet_NaN();
    if (fault == 2) e.metadata.presence = 256;
    if (fault == 3) {
      assert(r.offer(e) == Offer::Accepted);
      e = submission(1, 1, 102);
      e.metadata.presence = ControllerTime;
    }
    if (fault == 4) e.torque_presence = 8;
    if (fault == 5) {
      assert(r.offer(e) == Offer::Accepted);
      e = submission(1, 1, 102);
      e.torque_presence = MeasuredTorque;
    }
    if (fault == 6) e.signal_presence = 2;
    if (fault == 7) {
      assert(r.offer(e) == Offer::Accepted);
      e = submission(1, 1, 102);
      e.signal_presence = CommandedJointPosition;
    }
    assert(r.offer(e) == Offer::Rejected);
    drain(r, sink);
    const auto s = r.status();
    assert(s.reason == Reason::InvalidEvent && s.rejected == 1);
    assert(s.write_uncertain == 0 && s.abandoned == 0);
    assert(s.written == (fault == 3 || fault == 5 || fault == 7 ? 1 : 0));
  }
}
void ten_second_capture_uses_configured_event_budget() {
  Recorder r({7, 10000000000ULL, 50, 24096, 4});
  Sink sink;
  assert(r.start(7, 100));
  for (uint64_t i = 1; i <= 10000; ++i) {
    const auto t = 110 + (i - 1) * 1000000;
    assert(r.offer(state(i, t)) == Offer::Accepted);
    assert(r.offer(submission(i, i, t + 100)) == Offer::Accepted);
    while (r.writer_step(sink)) {}
  }
  r.tick(10000000110ULL);
  drain(r, sink);
  assert(r.status().reason == Reason::Duration);
  assert(r.status().written == 20000 && r.status().rejected == 0);
}
int main() {
  ten_second_capture_uses_configured_event_budget();
  malformed_payload_is_rejected_before_the_writer();
  owns_records_and_reports_overflow();
  duration_is_first_state_and_duplicate_start_does_not_extend_it();
  silent_expiry_and_cancellation();
  bad_associations_are_not_written();
  failure_accounting_and_finish();
  wraparound_limits_and_raw_anomalies();
  stale_clock_and_source_policy();
  earlier_tick_snapshot_does_not_fault_a_new_first_state();
  invalid_configuration();
}
