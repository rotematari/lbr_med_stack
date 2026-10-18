#include "lbr_acquisition/capture_encoding.hpp"

#include <utility>

namespace lbr_acquisition {
OfflineEvent encode_capture_event(const capture::Event &e,
                                  uint64_t expected_run_id,
                                  uint64_t latest_state) {
    if (!expected_run_id)
      throw InvalidInput("invalid run ID or capture batch size");
    if (e.run_id != expected_run_id)
      throw InvalidInput("capture batch mixes run IDs");
    const auto &m = e.metadata;
    constexpr uint32_t known = capture::ControllerTime | capture::SamplePeriod |
                               capture::Session | capture::Quality |
                               capture::Control | capture::Command;
    if (m.presence & ~known)
      throw InvalidInput("unsupported capture metadata presence");
    if (e.kind == capture::Kind::State) {
      if (e.state_id != e.id)
        throw InvalidInput("capture State identity mismatch");
      lbr::acquisition::v1::State s;
      s.set_schema_version(1);
      s.set_state_id(e.id);
      s.set_host_monotonic_ns(e.host_monotonic_ns);
      s.set_nonfinite_joint_mask(e.nonfinite_joint_mask);
      for (double q : e.position)
        s.add_q_measured_rad(q);
      if (m.presence & capture::ControllerTime) {
        s.set_controller_sec(m.controller_sec);
        s.set_controller_nsec(m.controller_nsec);
      }
      if (m.presence & capture::SamplePeriod)
        s.set_reported_sample_period_s(m.reported_sample_period_s);
      if (m.presence & capture::Session)
        s.set_session_state(m.session_state);
      if (m.presence & capture::Quality)
        s.set_connection_quality(m.connection_quality);
      if (m.presence & capture::Control)
        s.set_control_mode(m.control_mode);
      if (m.presence & capture::Command)
        s.set_client_command_mode(m.client_command_mode);
      return s;
    } else if (e.kind == capture::Kind::Submission) {
      if (m.presence || !latest_state || e.state_id != latest_state)
        throw InvalidInput(
            "invalid capture Submission metadata or association");
      lbr::acquisition::v1::Submission s;
      s.set_schema_version(1);
      s.set_submission_id(e.id);
      s.set_state_id(e.state_id);
      s.set_host_monotonic_ns(e.host_monotonic_ns);
      s.set_nonfinite_joint_mask(e.nonfinite_joint_mask);
      for (double q : e.position)
        s.add_q_submitted_rad(q);
      return s;
    } else {
      throw InvalidInput("unsupported capture event kind");
    }
}
std::vector<OfflineEvent>
encode_capture_batch(const std::vector<capture::Event> &events,
                     uint64_t expected_run_id) {
  if (!expected_run_id || events.size() > 10000)
    throw InvalidInput("invalid run ID or capture batch size");
  std::vector<OfflineEvent> output;
  output.reserve(events.size());
  uint64_t latest_state = 0;
  for (const auto &e : events) {
    output.emplace_back(encode_capture_event(e, expected_run_id, latest_state));
    if (e.kind == capture::Kind::State)
      latest_state = e.id;
  }
  // Quality anomalies remain raw and are reported by the existing writer.
  // Structural faults throw; local output is discarded without partial return.
  validate_events(output);
  return output;
}
} // namespace lbr_acquisition
