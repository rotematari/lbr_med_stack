// PHD-FRI-RECORDER: ROS control of numbered recordings on one FRI connection.
#pragma once
#include "lbr_acquisition/fri_recording_manager.hpp"
#include <atomic>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>

namespace rclcpp { class Node; }

namespace lbr_acquisition_ros {
// Recorder safety limits from config/recording/limits.yaml (see that file).
struct RecordingLimits {
  double max_duration_cap_s = 300.0;
  double default_first_sample_timeout_s = 5.0;
  int poll_period_ms = 50;
};
// Strict: unknown keys, missing keys or out-of-range values throw std::invalid_argument.
RecordingLimits load_limits(const std::filesystem::path &file);

// Shared by the driver App (FRI side) and RecordingServices (owner side).
// `manager` is null when recording control is disabled; `disabled_reason` says why.
struct RecordingRuntime {
  std::shared_ptr<lbr_acquisition::FriRecordingManager> manager;
  std::filesystem::path root, descriptor, limits_file;
  RecordingLimits limits;
  std::string disabled_reason;
  // Set by the FRI run thread after its last hook call; read by the owner timer.
  std::atomic<bool> producer_exited{false};
};

// Reads LBR_RECORDING_ROOT and optional LBR_RECORDING_CONFIG. Returns a disabled
// runtime (null manager, explanatory reason) when LBR_RECORDING_ROOT is unset or the
// one-shot LBR_RECORDING_DIRECTORY mode is active. Throws std::invalid_argument for
// both variables set, a relative/non-directory root or an invalid limits file.
// `default_limits` is called only when the root is set and LBR_RECORDING_CONFIG is not.
std::shared_ptr<RecordingRuntime>
runtime_from_environment(const std::filesystem::path &descriptor,
                         const std::function<std::filesystem::path()> &default_limits);

// Four services and a poll timer on `node`, all in ONE MutuallyExclusive callback
// group: this group is the manager's single owner. Handlers never wait for robot data.
//   ~/recording/prepare  ~/recording/start  ~/recording/stop  ~/recording/status
// Callbacks keep the shared state alive, so destroying this object while the executor
// runs a callback is safe.
class RecordingServices {
public:
  RecordingServices(const std::shared_ptr<rclcpp::Node> &node,
                    std::shared_ptr<RecordingRuntime> runtime);
  ~RecordingServices();
  RecordingServices(const RecordingServices &) = delete;
  RecordingServices &operator=(const RecordingServices &) = delete;
  struct Core;
private:
  std::shared_ptr<Core> core_;
};
} // namespace lbr_acquisition_ros
