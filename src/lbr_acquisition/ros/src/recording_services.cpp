// PHD-FRI-RECORDER: ROS glue owning FriRecordingManager from one callback group.
#include "lbr_acquisition_ros/recording_services.hpp"
#include "lbr_recording_interfaces/srv/prepare_recording.hpp"
#include "lbr_recording_interfaces/srv/recording_command.hpp"
#include "lbr_recording_interfaces/srv/recording_status.hpp"
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <set>
#include <stdexcept>

namespace lbr_acquisition_ros {
namespace c = lbr_acquisition::capture;
using lbr_acquisition::RecordingManagerPhase;
using PrepareSrv = lbr_recording_interfaces::srv::PrepareRecording;
using CommandSrv = lbr_recording_interfaces::srv::RecordingCommand;
using StatusSrv = lbr_recording_interfaces::srv::RecordingStatus;

namespace {
const char *reason_name(c::Reason r) {
  static const char *names[] = {"",           "Manual",       "Duration",   "NoData",
                                "Overflow",   "InvalidEvent", "EventLimit", "SourceUnavailable",
                                "ClockRegression", "WriterFailure"};
  const auto i = static_cast<size_t>(r);
  return i < sizeof(names) / sizeof(names[0]) ? names[i] : "Unknown";
}
uint8_t phase_code(RecordingManagerPhase p) {
  switch (p) {
  case RecordingManagerPhase::Idle: return StatusSrv::Response::IDLE;
  case RecordingManagerPhase::Prepared: return StatusSrv::Response::PREPARED;
  case RecordingManagerPhase::Armed: return StatusSrv::Response::ARMED;
  case RecordingManagerPhase::Recording: return StatusSrv::Response::RECORDING;
  case RecordingManagerPhase::Stopping: return StatusSrv::Response::STOPPING;
  case RecordingManagerPhase::Finalized: return StatusSrv::Response::FINALIZED;
  case RecordingManagerPhase::Cancelled: return StatusSrv::Response::CANCELLED;
  case RecordingManagerPhase::Failed: return StatusSrv::Response::FAILED;
  }
  return StatusSrv::Response::FAILED;
}
const char *phase_name(RecordingManagerPhase p) {
  static const char *names[] = {"Idle", "Prepared", "Armed", "Recording",
                                "Stopping", "Finalized", "Cancelled", "Failed"};
  return names[static_cast<size_t>(p)];
}
double positive_seconds(const YAML::Node &node, const char *key, double max) {
  const auto value = node[key].as<double>();
  if (!std::isfinite(value) || value <= 0. || value > max)
    throw std::invalid_argument(std::string(key) + " must be in (0, " + std::to_string(max) + "]");
  return value;
}
} // namespace

RecordingLimits load_limits(const std::filesystem::path &file) {
  YAML::Node root;
  try { root = YAML::LoadFile(file.string()); }
  catch (const std::exception &e) {
    throw std::invalid_argument("cannot read recording limits " + file.string() + ": " + e.what());
  }
  const std::set<std::string> keys{"max_duration_cap_s", "default_first_sample_timeout_s",
                                   "poll_period_ms"};
  if (!root.IsMap()) throw std::invalid_argument(file.string() + ": expected a mapping");
  for (const auto &entry : root)
    if (!keys.count(entry.first.as<std::string>()))
      throw std::invalid_argument(file.string() + ": unknown key '" +
                                  entry.first.as<std::string>() + "'");
  for (const auto &key : keys)
    if (!root[key]) throw std::invalid_argument(file.string() + ": missing key '" + key + "'");
  RecordingLimits limits;
  try {
    // 3600 s bounds the derived event budget (2 events/ms) well inside 64-bit counters.
    limits.max_duration_cap_s = positive_seconds(root, "max_duration_cap_s", 3600.);
    limits.default_first_sample_timeout_s =
        positive_seconds(root, "default_first_sample_timeout_s", 600.);
    limits.poll_period_ms = root["poll_period_ms"].as<int>();
  } catch (const YAML::Exception &e) {
    throw std::invalid_argument(file.string() + ": " + e.what());
  }
  if (limits.poll_period_ms < 1 || limits.poll_period_ms > 1000)
    throw std::invalid_argument(file.string() + ": poll_period_ms must be in [1, 1000]");
  return limits;
}

std::shared_ptr<RecordingRuntime>
runtime_from_environment(const std::filesystem::path &descriptor,
                         const std::function<std::filesystem::path()> &default_limits) {
  auto runtime = std::make_shared<RecordingRuntime>();
  runtime->descriptor = descriptor;
  const char *root = std::getenv("LBR_RECORDING_ROOT");
  const char *one_shot = std::getenv("LBR_RECORDING_DIRECTORY");
  if (root && one_shot)
    throw std::invalid_argument("Set only one of LBR_RECORDING_ROOT (recording services) and "
                                "LBR_RECORDING_DIRECTORY (one-shot recording)");
  if (one_shot) {
    runtime->disabled_reason = "Recording services are disabled: one-shot LBR_RECORDING_DIRECTORY "
                               "mode is active for this driver session.";
    return runtime;
  }
  if (!root || !*root) {
    runtime->disabled_reason =
        "Recording services are disabled because LBR_RECORDING_ROOT is not set. Before launching "
        "the driver, run e.g. `export LBR_RECORDING_ROOT=/abs/path/session-$(date +%Y%m%d-%H%M%S)`; "
        "runs are saved to <LBR_RECORDING_ROOT>/run-<run_id>.";
    return runtime;
  }
  runtime->root = root;
  if (!runtime->root.is_absolute())
    throw std::invalid_argument("LBR_RECORDING_ROOT must be an absolute path");
  std::error_code ec;
  std::filesystem::create_directories(runtime->root, ec);
  if (ec || !std::filesystem::is_directory(runtime->root))
    throw std::invalid_argument("LBR_RECORDING_ROOT is not a usable directory: " +
                                runtime->root.string() + (ec ? " (" + ec.message() + ")" : ""));
  const char *config = std::getenv("LBR_RECORDING_CONFIG");
  runtime->limits_file = config && *config ? std::filesystem::path(config) : default_limits();
  runtime->limits = load_limits(runtime->limits_file);
  runtime->manager = std::make_shared<lbr_acquisition::FriRecordingManager>();
  return runtime;
}

struct RecordingServices::Core {
  std::shared_ptr<RecordingRuntime> runtime;
  rclcpp::Logger logger;
  rclcpp::CallbackGroup::SharedPtr group;
  rclcpp::Service<PrepareSrv>::SharedPtr prepare;
  rclcpp::Service<CommandSrv>::SharedPtr start, stop;
  rclcpp::Service<StatusSrv>::SharedPtr status;
  rclcpp::TimerBase::SharedPtr timer;
  bool producer_reported = false;
  uint64_t announced = 0; // Last run whose retirement was logged.

  explicit Core(std::shared_ptr<RecordingRuntime> r, rclcpp::Logger l)
      : runtime(std::move(r)), logger(std::move(l)) {}
  lbr_acquisition::FriRecordingManager *manager() const { return runtime->manager.get(); }
  std::filesystem::path directory(uint64_t id) const {
    return runtime->root / ("run-" + std::to_string(id));
  }
  // Explain a manager refusal from its current state.
  std::string refusal(uint64_t id) const {
    const auto s = manager()->status();
    if (runtime->producer_exited.load() || s.source_failed)
      return "FRI source has stopped in this driver session; restart the driver to record again";
    if (s.owns_run) return "run " + std::to_string(s.run_id) + " is still " + phase_name(s.phase);
    if (s.phase == RecordingManagerPhase::Failed)
      return "run " + std::to_string(s.run_id) +
             " failed; recording is blocked until the driver restarts (inspect its files)";
    if (s.run_id && id <= s.run_id)
      return "run_id must be greater than the previous run_id " + std::to_string(s.run_id);
    return "request rejected for run " + std::to_string(id);
  }

  void on_prepare(const PrepareSrv::Request &q, PrepareSrv::Response &a) {
    if (!manager()) { a.message = runtime->disabled_reason; return; }
    const auto cap = runtime->limits.max_duration_cap_s;
    if (!q.run_id) { a.message = "run_id must be nonzero"; return; }
    if (!std::isfinite(q.max_duration_s) || q.max_duration_s <= 0. || q.max_duration_s > cap) {
      a.message = "max_duration_s must be in (0, " + std::to_string(cap) + "] (max_duration_cap_s in " +
                  runtime->limits_file.string() + ")";
      return;
    }
    const double timeout = q.first_sample_timeout_s > 0. ? q.first_sample_timeout_s
                                                         : runtime->limits.default_first_sample_timeout_s;
    if (!std::isfinite(timeout) || timeout > 600.) {
      a.message = "first_sample_timeout_s must be at most 600";
      return;
    }
    if (q.label.size() > 200) { a.message = "label is longer than 200 characters"; return; }
    const auto dir = directory(q.run_id);
    std::error_code ec;
    if (std::filesystem::exists(dir, ec) ||
        std::filesystem::exists(dir.string() + ".capture.json", ec)) {
      a.message = "output already exists: " + dir.string();
      return;
    }
    const auto duration_ns = static_cast<uint64_t>(std::llround(q.max_duration_s * 1e9));
    const auto timeout_ns = static_cast<uint64_t>(std::llround(timeout * 1e9));
    if (!duration_ns || !timeout_ns) {
      a.message = "max_duration_s and first_sample_timeout_s must be at least 1 ns";
      return;
    }
    const auto cycles = duration_ns / 1000000 + (duration_ns % 1000000 != 0);
    const c::Config config{q.run_id, duration_ns, timeout_ns, cycles * 2 + 4096, 4096};
    if (!manager()->prepare(config, dir, runtime->descriptor)) {
      a.message = std::filesystem::exists(dir, ec)
          ? "preparing " + dir.string() + " failed; recording is blocked (inspect partial files)"
          : refusal(q.run_id);
      return;
    }
    try {
      YAML::Emitter out;
      out << YAML::BeginMap << YAML::Key << "run_id" << YAML::Value << q.run_id
          << YAML::Key << "label" << YAML::Value << YAML::DoubleQuoted << q.label
          << YAML::Key << "max_duration_s" << YAML::Value << q.max_duration_s
          << YAML::Key << "first_sample_timeout_s" << YAML::Value << timeout
          << YAML::Key << "limits_file" << YAML::Value << runtime->limits_file.string()
          << YAML::EndMap;
      std::ofstream file;
      file.exceptions(std::ios::failbit | std::ios::badbit);
      file.open(dir / "request.yaml");
      file << out.c_str() << '\n';
    } catch (const std::exception &e) {
      manager()->request_stop(q.run_id); // Cancels the prepared run; the timer retires it.
      a.message = std::string("could not write request.yaml; run cancelled: ") + e.what();
      return;
    }
    a.accepted = true;
    a.directory = dir.string();
    a.message = "prepared; call start, then wait for status RECORDING";
    RCLCPP_INFO(logger, "Recording run %lu prepared in %s", static_cast<unsigned long>(q.run_id),
                dir.c_str());
  }
  void on_start(const CommandSrv::Request &q, CommandSrv::Response &a) {
    if (!manager()) { a.message = runtime->disabled_reason; return; }
    a.accepted = manager()->start(q.run_id);
    a.message = a.accepted ? "start accepted; wait for status RECORDING" : refusal(q.run_id);
  }
  void on_stop(const CommandSrv::Request &q, CommandSrv::Response &a) {
    if (!manager()) { a.message = runtime->disabled_reason; return; }
    a.accepted = manager()->request_stop(q.run_id);
    a.message = a.accepted ? "stop recognized; wait for status retired (FINALIZED, FAILED, or "
                             "CANCELLED if never started)"
                           : "run_id does not match the current or last run";
  }
  void on_status(const StatusSrv::Request &q, StatusSrv::Response &a) {
    if (!manager()) { a.message = runtime->disabled_reason; return; }
    const auto s = manager()->status();
    a.known = s.run_id && (q.run_id == 0 || q.run_id == s.run_id);
    if (!a.known) { a.message = "no current or last run with this run_id"; return; }
    a.run_id = s.run_id;
    a.phase = phase_code(s.phase);
    a.retired = s.retired;
    a.attached = s.attached;
    a.has_first_sample = s.session.capture.has_first;
    a.source_failed = s.source_failed;
    a.enqueued = s.session.capture.enqueued;
    a.written = s.session.capture.written;
    a.reason = reason_name(s.session.capture.reason);
    a.directory = directory(s.run_id).string();
    a.message = phase_name(s.phase);
  }
  void on_timer() {
    auto *m = manager();
    if (!m) return;
    if (runtime->producer_exited.load() && !producer_reported) {
      m->producer_stopped(); // The FRI thread has made its last hook call.
      producer_reported = true;
    }
    const auto s = m->poll();
    if (s.owns_run && s.phase == RecordingManagerPhase::Stopping) m->finish(s.run_id);
    const auto done = m->status();
    if (done.retired && done.run_id != announced) {
      announced = done.run_id;
      RCLCPP_INFO(logger, "Recording run %lu %s: reason=%s written=%lu",
                  static_cast<unsigned long>(done.run_id), phase_name(done.phase),
                  reason_name(done.session.capture.reason),
                  static_cast<unsigned long>(done.session.capture.written));
    }
  }
};

namespace {
// Typed service callback; it holds a strong Core reference only while it runs.
template <class Srv>
std::function<void(std::shared_ptr<typename Srv::Request>, std::shared_ptr<typename Srv::Response>)>
handler(std::weak_ptr<RecordingServices::Core> weak,
        void (RecordingServices::Core::*member)(const typename Srv::Request &,
                                                typename Srv::Response &)) {
  return [weak, member](std::shared_ptr<typename Srv::Request> request,
                        std::shared_ptr<typename Srv::Response> response) {
    if (auto core = weak.lock()) ((*core).*member)(*request, *response);
  };
}
} // namespace

RecordingServices::RecordingServices(const std::shared_ptr<rclcpp::Node> &node,
                                     std::shared_ptr<RecordingRuntime> runtime)
    : core_(std::make_shared<Core>(std::move(runtime), node->get_logger().get_child("recording"))) {
  auto &k = *core_;
  if (!k.manager()) RCLCPP_ERROR(k.logger, "%s", k.runtime->disabled_reason.c_str());
  else
    RCLCPP_INFO(k.logger, "Recording services enabled: root=%s limits=%s cap=%.1f s",
                k.runtime->root.c_str(), k.runtime->limits_file.c_str(),
                k.runtime->limits.max_duration_cap_s);
  k.group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  const std::weak_ptr<Core> weak = core_;
  const auto qos = rclcpp::ServicesQoS();
  k.prepare = node->create_service<PrepareSrv>("~/recording/prepare",
      handler<PrepareSrv>(weak, &Core::on_prepare), qos, k.group);
  k.start = node->create_service<CommandSrv>("~/recording/start",
      handler<CommandSrv>(weak, &Core::on_start), qos, k.group);
  k.stop = node->create_service<CommandSrv>("~/recording/stop",
      handler<CommandSrv>(weak, &Core::on_stop), qos, k.group);
  k.status = node->create_service<StatusSrv>("~/recording/status",
      handler<StatusSrv>(weak, &Core::on_status), qos, k.group);
  k.timer = node->create_wall_timer(std::chrono::milliseconds(k.runtime->limits.poll_period_ms),
                                    [weak] { if (auto core = weak.lock()) core->on_timer(); }, k.group);
}
RecordingServices::~RecordingServices() {
  // Stop new callbacks; one already running keeps Core (and the manager) alive.
  core_->timer->cancel();
  core_->timer.reset();
  core_->prepare.reset();
  core_->start.reset();
  core_->stop.reset();
  core_->status.reset();
}
} // namespace lbr_acquisition_ros
