#pragma once
#include <array>
#include <cstdint>
#include <type_traits>

namespace lbr_acquisition::capture {
enum class Kind : uint8_t { State, Submission };
enum Presence : uint32_t {
  ControllerTime = 1,
  SamplePeriod = 2,
  Session = 4,
  Quality = 8,
  Control = 16,
  Command = 32,
  Safety = 64,
  Tracking = 128
};
struct StateMetadata {
  uint32_t presence = 0;
  uint32_t controller_sec = 0, controller_nsec = 0;
  uint32_t session_state = 0, connection_quality = 0;
  uint32_t control_mode = 0, client_command_mode = 0;
  double reported_sample_period_s = 0;
  uint32_t safety_state = 0;
  double tracking_performance = 0;
};
enum TorquePresence : uint32_t {
  MeasuredTorque = 1,
  CommandedTorque = 2,
  EstimatedExternalTorque = 4
};
enum SignalPresence : uint32_t { CommandedJointPosition = 1 };
struct Event {
  Kind kind = Kind::State;
  uint64_t run_id = 0, id = 0, state_id = 0, host_monotonic_ns = 0;
  std::array<double, 7> position{};
  uint32_t nonfinite_joint_mask = 0;
  StateMetadata metadata{};
  // State only. A clear bit means unavailable; dormant array values are ignored.
  uint32_t torque_presence = 0;
  std::array<double, 7> tau_measured_nm{}, tau_commanded_nm{}, tau_external_estimated_nm{};
  // State only, same convention: KUKA's last commanded joint positions.
  uint32_t signal_presence = 0;
  std::array<double, 7> q_commanded_rad{};
};
static_assert(std::is_trivially_copyable_v<Event>);
static_assert(std::is_nothrow_copy_constructible_v<Event>);
static_assert(std::is_trivially_destructible_v<Event>);
} // namespace lbr_acquisition::capture
