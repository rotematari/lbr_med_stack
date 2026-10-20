#include "lbr_acquisition/capture_encoding.hpp"
#include "lbr_acquisition/recorder.hpp"
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
using namespace lbr_acquisition;
namespace c = lbr_acquisition::capture;
using State = lbr::acquisition::v1::State;
using Submission = lbr::acquisition::v1::Submission;
constexpr uint64_t H = 9007199254740992ULL;
c::Event state(uint64_t id, uint64_t time) {
  c::Event e{};
  e.run_id = 17;
  e.id = e.state_id = id;
  e.host_monotonic_ns = time;
  e.position = {.125, -0., -.25, .375, -.5, .625, -.75};
  return e;
}
std::vector<c::Event> fixture() {
  auto a = state(10, H), b = state(11, H + 1), s = state(20, H + 2);
  // Dormant values must not manufacture available fields.
  a.metadata.controller_sec = 123;
  a.metadata.session_state = 99;
  b.position = {.25, .125, -.375, .5, -.625, .75, -.875};
  b.metadata = {63, 0, 0, 4, 3, 2, 1, .001};
  // Values in absent fields must never manufacture a measurement.
  a.tau_measured_nm.fill(42);
  a.tau_commanded_nm.fill(43);
  a.tau_external_estimated_nm.fill(44);
  b.torque_presence = c::MeasuredTorque | c::CommandedTorque | c::EstimatedExternalTorque;
  b.tau_measured_nm = {1.25, -0., -2.5, 3.75, -5., 6.25, -7.5};
  b.tau_commanded_nm = {11.25, 10., -12.5, 13.75, -15., 16.25, -17.5};
  b.tau_external_estimated_nm = {.125, -0., -.25, .375, -.5, .625, -.75};
  s.kind = c::Kind::Submission;
  s.state_id = 11;
  s.position = {.5, -.125, .25, -.375, .5, -.625, .75};
  return {a, b, s};
}
uint64_t bits(double v) {
  uint64_t b;
  std::memcpy(&b, &v, sizeof(b));
  return b;
}
template <class F> void invalid(F f) {
  bool caught = false;
  try {
    f();
  } catch (const InvalidInput &) {
    caught = true;
  }
  assert(caught);
}
void fidelity_and_presence() {
  auto in = fixture();
  auto out = encode_capture_batch(in, 17);
  assert(out.size() == 3);
  const auto &a = std::get<State>(out[0]), &b = std::get<State>(out[1]);
  const auto &s = std::get<Submission>(out[2]);
  assert(a.state_id() == 10 && b.state_id() == 11 && s.submission_id() == 20 &&
         s.state_id() == 11);
  assert(a.host_monotonic_ns() == H && b.host_monotonic_ns() == H + 1 &&
         s.host_monotonic_ns() == H + 2);
  assert(!a.has_controller_sec() && !a.has_controller_nsec() &&
         !a.has_session_state());
  assert(b.has_controller_sec() && b.controller_sec() == 0 &&
         b.has_controller_nsec() && b.controller_nsec() == 0);
  assert(b.reported_sample_period_s() == .001 && b.session_state() == 4 &&
         b.connection_quality() == 3);
  assert(b.control_mode() == 2 && b.client_command_mode() == 1);
  for (int j = 0; j < 7; ++j) {
    assert(bits(a.q_measured_rad(j)) == bits(in[0].position[j]));
    assert(bits(b.q_measured_rad(j)) == bits(in[1].position[j]));
    assert(bits(s.q_submitted_rad(j)) == bits(in[2].position[j]));
    assert(bits(b.tau_measured_nm(j)) == bits(in[1].tau_measured_nm[j]));
    assert(bits(b.tau_commanded_nm(j)) == bits(in[1].tau_commanded_nm[j]));
    assert(bits(b.tau_external_estimated_nm(j)) == bits(in[1].tau_external_estimated_nm[j]));
  }
  assert(a.tau_measured_nm_size() == 0 && a.tau_commanded_nm_size() == 0 &&
         a.tau_external_estimated_nm_size() == 0);
  assert(a.schema_version() == 1 && b.schema_version() == 1 &&
         s.schema_version() == 1);
  assert(a.nonfinite_joint_mask() == 0 && s.nonfinite_joint_mask() == 0);
  in[0].position.fill(42);
  assert(a.q_measured_rad(0) == .125); // independent ownership
  for (uint32_t bit = 1; bit <= 32; bit <<= 1) {
    auto e = fixture()[1];
    e.metadata.presence = bit;
    auto batch = encode_capture_batch({e}, 17);
    const auto &m = std::get<State>(batch[0]);
    assert(m.has_controller_sec() == (bit == 1) &&
           m.has_controller_nsec() == (bit == 1));
    assert(m.has_reported_sample_period_s() == (bit == 2));
    assert(m.has_session_state() == (bit == 4));
    assert(m.has_connection_quality() == (bit == 8));
    assert(m.has_control_mode() == (bit == 16));
    assert(m.has_client_command_mode() == (bit == 32));
  }
}
void torque_availability_and_validation() {
  for (uint32_t presence = 0; presence < 8; ++presence) {
    auto e = fixture()[1];
    e.torque_presence = presence;
    auto out = encode_capture_batch({e}, 17);
    const auto &s = std::get<State>(out[0]);
    assert(s.tau_measured_nm_size() == ((presence & 1) ? 7 : 0));
    assert(s.tau_commanded_nm_size() == ((presence & 2) ? 7 : 0));
    assert(s.tau_external_estimated_nm_size() == ((presence & 4) ? 7 : 0));
  }
  auto e = fixture()[1];
  e.torque_presence = 8;
  invalid([&] { encode_capture_batch({e}, 17); });
  auto events = fixture();
  events[2].torque_presence = c::MeasuredTorque;
  invalid([&] { encode_capture_batch(events, 17); });
  // Native writer validation must reject partial arrays, and retain/report
  // nonfinite data separately for each torque signal.
  for (int signal = 0; signal < 3; ++signal) {
    auto out = encode_capture_batch({fixture()[1]}, 17);
    auto &s = std::get<State>(out[0]);
    auto *values = signal == 0 ? s.mutable_tau_measured_nm()
                  : signal == 1 ? s.mutable_tau_commanded_nm()
                                : s.mutable_tau_external_estimated_nm();
    values->Set(0, std::numeric_limits<double>::quiet_NaN());
    assert(validate_events(out).size() == 1);
    values->RemoveLast();
    invalid([&] { validate_events(out); });
  }
}
void invalid_batches() {
  assert(encode_capture_batch({}, 17).empty());
  invalid([] { encode_capture_batch({}, 0); });
  for (int which = 0; which < 9; ++which) {
    auto in = fixture();
    switch (which) {
    case 0:
      in[1].run_id = 18;
      break;
    case 1:
      in[1].kind = static_cast<c::Kind>(99);
      break;
    case 2:
      in[1].metadata.presence = 64;
      break;
    case 3:
      in[1].state_id = 9;
      break;
    case 4:
      in[2].metadata.presence = 1;
      break;
    case 5:
      in[2].state_id = 10;
      break; // seen but not latest
    case 6:
      in[1].nonfinite_joint_mask = 1;
      break;
    case 7:
      in[1].id = in[1].state_id = 10;
      break;
    case 8:
      in[2].state_id = 999;
      break;
    }
    invalid([&] { encode_capture_batch(in, 17); });
    assert(in[0].id == 10 && bits(in[0].position[1]) == bits(-0.));
  }
  invalid([] { encode_capture_batch({fixture()[2]}, 17); });
}
void limits_and_anomalies() {
  std::vector<c::Event> in;
  in.reserve(10001);
  for (uint64_t id = 1; id <= 10000; ++id)
    in.push_back(state(id, H + id));
  assert(encode_capture_batch(in, 17).size() == 10000);
  in.push_back(state(10001, H + 10001));
  invalid([&] { encode_capture_batch(in, 17); });
  auto e = fixture()[1];
  e.position[0] = std::numeric_limits<double>::quiet_NaN();
  e.position[1] = std::numeric_limits<double>::infinity();
  e.position[2] = -e.position[1];
  e.nonfinite_joint_mask = 7;
  e.metadata.controller_nsec = UINT32_MAX;
  e.metadata.session_state = 99;
  auto out = encode_capture_batch({e}, 17);
  const auto &m = std::get<State>(out[0]);
  assert(std::isnan(m.q_measured_rad(0)) && std::isinf(m.q_measured_rad(1)));
  assert(!std::signbit(m.q_measured_rad(1)) &&
         std::signbit(m.q_measured_rad(2)));
  assert(m.nonfinite_joint_mask() == 7 && m.controller_nsec() == UINT32_MAX &&
         m.session_state() == 99);
  assert(!validate_events(out).empty());
}
struct Collector : c::Writer {
  std::vector<c::Event> events;
  void write(const c::Event &e) override { events.push_back(e); }
  void finish() override {}
};
void make_fixture(const char *dir, const char *descriptor) {
  c::Recorder recorder({17, 1000, 1000, 10, 4});
  Collector sink;
  sink.events.reserve(3);
  assert(recorder.start(17, H - 1));
  for (const auto &e : fixture())
    assert(recorder.offer(e) == c::Offer::Accepted);
  assert(recorder.stop(17));
  for (unsigned i = 0; i < 8 && !recorder.status().writer_done; ++i)
    recorder.writer_step(sink);
  assert(recorder.status().phase == c::Phase::Complete &&
         sink.events.size() == 3);
  write_fixture(dir, encode_capture_batch(sink.events, 17), descriptor);
}
int main(int argc, char **argv) {
  try {
    if (argc == 4 && std::string(argv[1]) == "--fixture") {
      make_fixture(argv[2], argv[3]);
      return 0;
    }
    if (argc != 1)
      return 2;
    fidelity_and_presence();
    torque_availability_and_validation();
    invalid_batches();
    limits_and_anomalies();
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
