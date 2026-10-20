// PHD-FRI-RECORDER: offline check of the ROS recording services over the real driver
// client and an in-memory FRI connection. The test thread is the FRI producer; a
// multithreaded executor serves the services, as in ros2_control_node.
#include "fri_fixture.hpp"
#include "lbr_acquisition_ros/recording_services.hpp"
#include "lbr_recording_interfaces/srv/prepare_recording.hpp"
#include "lbr_recording_interfaces/srv/recording_command.hpp"
#include "lbr_recording_interfaces/srv/recording_status.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <thread>

using namespace fri_fixture;
using namespace std::chrono_literals;
namespace fs = std::filesystem;
using lbr_acquisition_ros::RecordingRuntime;
using lbr_acquisition_ros::RecordingServices;
using Prepare = lbr_recording_interfaces::srv::PrepareRecording;
using Command = lbr_recording_interfaces::srv::RecordingCommand;
using Status = lbr_recording_interfaces::srv::RecordingStatus;

namespace {
std::string read(const fs::path &path) {
  std::ifstream file(path);
  return {std::istreambuf_iterator<char>{file}, {}};
}
bool throws(const std::function<void()> &call) {
  try { call(); } catch (const std::invalid_argument &) { return true; }
  return false;
}
void write(const fs::path &path, const std::string &text) { std::ofstream(path) << text; }

struct Client {
  rclcpp::Node::SharedPtr node;
  rclcpp::Client<Prepare>::SharedPtr prepare;
  rclcpp::Client<Command>::SharedPtr start, stop;
  rclcpp::Client<Status>::SharedPtr status;
  explicit Client(const std::string &server) : node(rclcpp::Node::make_shared(server + "_client")) {
    const auto base = "/" + server + "/recording/";
    prepare = node->create_client<Prepare>(base + "prepare");
    start = node->create_client<Command>(base + "start");
    stop = node->create_client<Command>(base + "stop");
    status = node->create_client<Status>(base + "status");
  }
  template <class S> typename S::Response::SharedPtr call(const typename rclcpp::Client<S>::SharedPtr &c,
                                                         typename S::Request::SharedPtr request) {
    if (!c->wait_for_service(5s)) { CHECK(false); return std::make_shared<typename S::Response>(); }
    auto future = c->async_send_request(request);
    if (future.wait_for(5s) != std::future_status::ready) {
      CHECK(false);
      return std::make_shared<typename S::Response>();
    }
    return future.get();
  }
  Prepare::Response::SharedPtr prep(uint64_t id, double max_s, std::string label = "") {
    auto q = std::make_shared<Prepare::Request>();
    q->run_id = id; q->max_duration_s = max_s; q->label = std::move(label);
    return call<Prepare>(prepare, q);
  }
  Command::Response::SharedPtr command(const rclcpp::Client<Command>::SharedPtr &c, uint64_t id) {
    auto q = std::make_shared<Command::Request>();
    q->run_id = id;
    return call<Command>(c, q);
  }
  Status::Response::SharedPtr stat(uint64_t id) {
    auto q = std::make_shared<Status::Request>();
    q->run_id = id;
    return call<Status>(status, q);
  }
  // Poll status while `tick` drives FRI cycles, until `done` holds (5 s limit).
  Status::Response::SharedPtr until(uint64_t id, const std::function<bool(const Status::Response &)> &done,
                                    const std::function<void()> &tick) {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    auto s = stat(id);
    while (!done(*s) && std::chrono::steady_clock::now() < deadline) {
      if (tick) tick();
      std::this_thread::sleep_for(5ms);
      s = stat(id);
    }
    CHECK(done(*s));
    return s;
  }
};

void limits_cases(const fs::path &limits, const fs::path &root) {
  const auto ok = lbr_acquisition_ros::load_limits(limits);
  CHECK(ok.max_duration_cap_s == 300. && ok.default_first_sample_timeout_s == 5. &&
        ok.poll_period_ms == 50);
  const auto bad = root / "bad-limits.yaml";
  const std::string valid = "max_duration_cap_s: 300\ndefault_first_sample_timeout_s: 5\npoll_period_ms: 50\n";
  for (const auto &text : {valid + "extra: 1\n", std::string("max_duration_cap_s: 300\npoll_period_ms: 50\n"),
                           std::string("max_duration_cap_s: -1\ndefault_first_sample_timeout_s: 5\npoll_period_ms: 50\n"),
                           std::string("max_duration_cap_s: 300\ndefault_first_sample_timeout_s: 5\npoll_period_ms: 0\n"),
                           std::string("max_duration_cap_s: abc\ndefault_first_sample_timeout_s: 5\npoll_period_ms: 50\n"),
                           std::string("- not a map\n")}) {
    write(bad, text);
    CHECK(throws([&] { lbr_acquisition_ros::load_limits(bad); }));
  }
  CHECK(throws([&] { lbr_acquisition_ros::load_limits(root / "missing.yaml"); }));
}

void environment_cases(const fs::path &descriptor, const fs::path &limits, const fs::path &root) {
  bool default_used = false;
  auto fallback = [&] { default_used = true; return limits; };
  unsetenv("LBR_RECORDING_ROOT"); unsetenv("LBR_RECORDING_DIRECTORY"); unsetenv("LBR_RECORDING_CONFIG");
  auto off = lbr_acquisition_ros::runtime_from_environment(descriptor, fallback);
  CHECK(!off->manager && off->disabled_reason.find("LBR_RECORDING_ROOT") != std::string::npos);
  setenv("LBR_RECORDING_DIRECTORY", (root / "one-shot").c_str(), 1);
  auto one = lbr_acquisition_ros::runtime_from_environment(descriptor, fallback);
  CHECK(!one->manager && one->disabled_reason.find("one-shot") != std::string::npos);
  setenv("LBR_RECORDING_ROOT", (root / "both").c_str(), 1);
  CHECK(throws([&] { lbr_acquisition_ros::runtime_from_environment(descriptor, fallback); }));
  unsetenv("LBR_RECORDING_DIRECTORY");
  setenv("LBR_RECORDING_ROOT", "relative/root", 1);
  CHECK(throws([&] { lbr_acquisition_ros::runtime_from_environment(descriptor, fallback); }));
  write(root / "a-file", "x");
  setenv("LBR_RECORDING_ROOT", (root / "a-file").c_str(), 1);
  CHECK(throws([&] { lbr_acquisition_ros::runtime_from_environment(descriptor, fallback); }));
  CHECK(!default_used); // Invalid roots fail before any limits lookup.
  setenv("LBR_RECORDING_ROOT", (root / "default-limits").c_str(), 1);
  auto on = lbr_acquisition_ros::runtime_from_environment(descriptor, fallback);
  CHECK(on->manager && default_used && fs::is_directory(root / "default-limits"));
  default_used = false;
  setenv("LBR_RECORDING_CONFIG", limits.c_str(), 1);
  on = lbr_acquisition_ros::runtime_from_environment(descriptor, fallback);
  CHECK(on->manager && !default_used && on->limits_file == limits);
  setenv("LBR_RECORDING_CONFIG", (root / "missing.yaml").c_str(), 1);
  CHECK(throws([&] { lbr_acquisition_ros::runtime_from_environment(descriptor, fallback); }));
  unsetenv("LBR_RECORDING_ROOT"); unsetenv("LBR_RECORDING_CONFIG");
}
} // namespace

int main(int argc, char **argv) {
  if (argc != 4) { std::cerr << "usage: descriptor output-root limits.yaml\n"; return 2; }
  const fs::path descriptor = argv[1], root = argv[2], limits = argv[3];
  fs::remove_all(root); // Dedicated build-tree fixture output only.
  fs::create_directories(root);
  rclcpp::init(argc, argv);
  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_FATAL);
  limits_cases(limits, root);
  environment_cases(descriptor, limits, root);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  { // Disabled: services exist and refuse with the configuration instruction.
    unsetenv("LBR_RECORDING_ROOT");
    auto runtime = lbr_acquisition_ros::runtime_from_environment(descriptor, [&] { return limits; });
    auto node = rclcpp::Node::make_shared("rec_disabled");
    auto services = std::make_unique<RecordingServices>(node, runtime);
    Client client("rec_disabled");
    executor.add_node(node); executor.add_node(client.node);
    std::thread spin([&] { executor.spin(); });
    const auto p = client.prep(1, 1.);
    CHECK(!p->accepted && p->message.find("LBR_RECORDING_ROOT") != std::string::npos);
    CHECK(!client.command(client.start, 1)->accepted);
    CHECK(client.stat(0)->message.find("LBR_RECORDING_ROOT") != std::string::npos);
    executor.cancel(); spin.join();
    executor.remove_node(node); executor.remove_node(client.node);
  }

  const auto session = root / "session";
  setenv("LBR_RECORDING_ROOT", session.c_str(), 1);
  setenv("LBR_RECORDING_CONFIG", limits.c_str(), 1);
  auto runtime = lbr_acquisition_ros::runtime_from_environment(descriptor, [&] { return limits; });
  unsetenv("LBR_RECORDING_ROOT"); unsetenv("LBR_RECORDING_CONFIG");
  auto node = rclcpp::Node::make_shared("rec");
  auto services = std::make_unique<RecordingServices>(node, runtime);
  Client client("rec");
  executor.add_node(node); executor.add_node(client.node);
  std::thread spin([&] { executor.spin(); });
  DriverRig rig(runtime->manager->adapter(), 400);
  unsigned steps = 0;
  auto tick = [&] { if (steps < 399) { rig.step(); ++steps; } };
  tick(); // Session-2 packet while idle.
  using R = Status::Response;
  auto phase_is = [](uint8_t p) { return [p](const R &s) { return s.phase == p; }; };

  // Run A then run B on one connection and one adapter.
  for (const uint64_t id : {5u, 9u}) {
    const auto p = client.prep(id, 10., "run \"" + std::to_string(id) + "\": quoted label");
    CHECK(p->accepted && p->directory == (session / ("run-" + std::to_string(id))).string());
    CHECK(read(fs::path(p->directory) / "request.yaml").find("quoted label") != std::string::npos);
    CHECK(client.stat(id)->phase == R::PREPARED);
    CHECK(client.command(client.start, id)->accepted);
    CHECK(!client.command(client.start, id)->accepted); // Duplicate start.
    CHECK(client.stat(id)->phase == R::ARMED);
    auto s = client.until(id, phase_is(R::RECORDING), tick);
    CHECK(s->has_first_sample && s->attached);
    // While recording: busy prepare, wrong-ID stop/status, and concurrent requests.
    CHECK(!client.prep(id + 1, 1.)->accepted);
    CHECK(!client.command(client.stop, id + 100)->accepted);
    CHECK(!client.stat(id + 100)->known);
    std::vector<rclcpp::Client<Status>::SharedFuture> burst;
    for (int i = 0; i < 8; ++i) { // Within the default services QoS depth (10).
      auto q = std::make_shared<Status::Request>(); q->run_id = id;
      burst.push_back(client.status->async_send_request(q).future.share());
    }
    for (int i = 0; i < 3; ++i) tick();
    for (auto &f : burst) CHECK(f.wait_for(5s) == std::future_status::ready && f.get()->known);
    CHECK(client.command(client.stop, id)->accepted);
    CHECK(client.command(client.stop, id)->accepted); // Duplicate stop is harmless.
    s = client.until(id, [](const R &r) { return r.retired; }, tick);
    CHECK(s->phase == R::FINALIZED && s->reason == "Manual" && s->written == s->enqueued);
    CHECK(s->enqueued >= 4 && fs::exists(session / ("run-" + std::to_string(id) + ".capture.json")));
    std::cout << "run " << id << ": " << s->message << " reason=" << s->reason
              << " written=" << s->written << '\n';
  }
  CHECK(client.stat(0)->run_id == 9 && !client.stat(5)->known);
  // Rejections: stale/duplicate IDs, cap, zero ID, long label, existing output.
  const auto again = client.prep(9, 1.); // Existing output is checked before the ID.
  CHECK(!again->accepted && again->message.find("already exists") != std::string::npos);
  const auto stale = client.prep(7, 1.);
  CHECK(!stale->accepted && stale->message.find("greater") != std::string::npos);
  const auto tiny = client.prep(20, 1e-12); // Rounds to 0 ns: refused, manager not latched.
  CHECK(!tiny->accepted && tiny->message.find("1 ns") != std::string::npos);
  const auto cap = client.prep(20, 300.5);
  CHECK(!cap->accepted && cap->message.find("max_duration_cap_s") != std::string::npos);
  CHECK(!client.prep(0, 1.)->accepted);
  CHECK(!client.prep(20, 1., std::string(201, 'x'))->accepted);
  fs::create_directories(session / "run-21");
  CHECK(client.prep(21, 1.)->message.find("already exists") != std::string::npos);
  CHECK(client.prep(22, 300.)->accepted); // At the cap; then cancel before start.
  CHECK(client.command(client.stop, 22)->accepted);
  CHECK(client.until(22, [](const R &r) { return r.retired; }, nullptr)->phase == R::CANCELLED);

  // Producer exit while recording: truthful failure, and no further runs.
  CHECK(client.prep(30, 10.)->accepted && client.command(client.start, 30)->accepted);
  client.until(30, phase_is(R::RECORDING), tick);
  runtime->manager->adapter()->source_stopped(); // What App::perform_work_ does at exit.
  runtime->producer_exited.store(true);
  const auto exited = client.until(30, [](const R &r) { return r.retired; }, nullptr);
  CHECK(exited->phase == R::FAILED && exited->reason == "SourceUnavailable" && exited->source_failed);
  CHECK(client.prep(31, 1.)->message.find("FRI source has stopped") != std::string::npos);

  services.reset(); // While the executor still spins.
  executor.cancel(); spin.join();
  std::cout << "FRI cycles stepped: " << steps << '\n';
  rclcpp::shutdown();
  if (failures) { std::cerr << failures << " failed checks\n"; return 1; }
  std::cout << "Recording services: all checks passed.\n";
}
