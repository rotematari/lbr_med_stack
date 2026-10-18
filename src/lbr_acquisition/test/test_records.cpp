#include "lbr_acquisition/offline_writer.hpp"
#include <cassert>
#include <cmath>
int main() {
  auto events = lbr_acquisition::synthetic_events();
  assert(events.size() == 5);
  auto s = std::get<lbr::acquisition::v1::State>(events[0]);
  assert(s.state_id() == 100 && !s.has_controller_sec());
  assert(s.q_measured_rad_size() == 7 && std::signbit(s.q_measured_rad(1)));
  auto z = std::get<lbr::acquisition::v1::State>(events[1]);
  assert(z.host_monotonic_ns() == 9007199254740993ULL);
  assert(z.has_controller_sec() && z.controller_sec() == 0);
  lbr_acquisition::validate_events(events);
  auto bad = events;
  std::get<lbr::acquisition::v1::State>(bad[1]).set_state_id(100);
  bool rejected = false;
  try {
    lbr_acquisition::validate_events(bad);
  } catch (const lbr_acquisition::InvalidInput &) {
    rejected = true;
  }
  assert(rejected);
  assert(lbr::acquisition::v1::State::descriptor()->full_name() ==
         "lbr.acquisition.v1.State");
  // Exercise writer-side validation independently from the Python reader.
  auto rejects = [](const std::vector<lbr_acquisition::OfflineEvent> &invalid) {
    bool threw = false;
    try {
      lbr_acquisition::validate_events(invalid);
    } catch (const lbr_acquisition::InvalidInput &) {
      threw = true;
    }
    assert(threw);
  };
  for (int which = 0; which < 8; ++which) {
    auto changed = events;
    auto &value = std::get<lbr::acquisition::v1::State>(changed[0]);
    switch (which) {
    case 0:
      value.clear_host_monotonic_ns();
      break;
    case 1:
      value.set_schema_version(2);
      break;
    case 2:
      value.set_state_id(0);
      break;
    case 3:
      value.set_controller_sec(0);
      break;
    case 4:
      value.add_q_measured_rad(0);
      break;
    case 5:
      value.set_nonfinite_joint_mask(128);
      break;
    case 6:
      value.GetReflection()->MutableUnknownFields(&value)->AddVarint(100, 1);
      break;
    case 7:
      std::get<lbr::acquisition::v1::Submission>(changed[2]).set_state_id(999);
      break;
    }
    rejects(changed);
  }
  auto anomalous = events;
  auto &raw = std::get<lbr::acquisition::v1::State>(anomalous[0]);
  raw.set_q_measured_rad(0, std::nan(""));
  raw.set_nonfinite_joint_mask(1);
  raw.set_controller_sec(0);
  raw.set_controller_nsec(1000000000);
  raw.set_reported_sample_period_s(-1);
  raw.set_control_mode(99);
  assert(lbr_acquisition::validate_events(anomalous).size() == 4);
  lbr::acquisition::v1::State empty;
  assert(!empty.IsInitialized());
}
