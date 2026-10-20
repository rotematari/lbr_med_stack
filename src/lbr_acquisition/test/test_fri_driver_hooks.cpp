// PHD-FRI-RECORDER: compact offline fixture for our additions to the real FRI driver.
#include "lbr_acquisition/fri_adapter.hpp"
#include "lbr_acquisition/fri_recording_manager.hpp"
#include "fri_fixture.hpp"
#include "lbr_acquisition/streaming_capture_session.hpp"
#include "lbr_fri_ros2/async_client.hpp"
#include "lbr_fri_ros2/app.hpp"
#include <algorithm>
#include <cstdlib>
#include <new>
#include "friClientApplication.h"
#include <rcutils/logging.h>
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

namespace c = lbr_acquisition::capture;
using namespace lbr_acquisition;
using namespace fri_fixture;
thread_local bool track_allocations = false;
thread_local size_t allocations = 0;
void *operator new(std::size_t size) {
  if (track_allocations) ++allocations;
  if (void *p = std::malloc(size ? size : 1)) return p;
  throw std::bad_alloc();
}
void operator delete(void *p) noexcept { if (track_allocations) ++allocations; std::free(p); }
void operator delete(void *p, std::size_t) noexcept { if (track_allocations) ++allocations; std::free(p); }

struct MemoryWriter : c::Writer {
  std::vector<c::Event> events;
  void write(const c::Event &e) override { events.push_back(e); }
  void finish() override {}
};
struct Result {
  std::vector<std::string> sent;
  std::vector<c::Event> events;
  std::vector<Joints> submitted, ros_position, ros_measured_torque, ros_external_torque;
  c::Status status;
};

Result run(bool enabled, bool fill_queue = false) {
  now = 1000000;
  c::Recorder recorder({1, 1000000000, 60000000000, 128, fill_queue ? 1u : 64u});
  CHECK(recorder.start(1, now));
  CaptureEndpoint endpoint{
    &recorder,
    [](void *p, const c::Event &e, bool eligible) noexcept {
      return static_cast<c::Recorder *>(p)->offer(e, eligible);
    },
    [](void *p, uint64_t id, c::Reason reason) noexcept {
      return static_cast<c::Recorder *>(p)->stop(id, reason);
    }};
  auto adapter = std::make_shared<RecorderAdapter>(1, endpoint, clock_ns);
  auto memory = std::make_unique<MemoryConnection>();
  auto *wire = memory.get();
  wire->inputs = {packet(0, 2), packet(1, 4), packet(2, 4), packet(3, 4, 2),
                  packet(4, 2), packet(5, 4)};
  std::unique_ptr<KUKA::FRI::IConnection> connection;
  if (enabled) connection = std::make_unique<ObservedConnection>(std::move(memory), adapter);
  else connection = std::move(memory);
  lbr_fri_ros2::CommandGuardParameters guard;
  guard.min_positions.fill(-2.); guard.max_positions.fill(2.);
  guard.max_velocities.fill(10000.); guard.max_torques.fill(1000.);
  lbr_fri_ros2::AsyncClient client(KUKA::FRI::POSITION, .01, guard, "default", {}, {.04, .04}, true);
  if (enabled) client.set_recorder(adapter);
  KUKA::FRI::ClientApplication app(*connection, client);
  MemoryWriter writer;
  Result result;
  for (unsigned i = 0; i < 6; ++i) {
    lbr_fri_idl::msg::LBRCommand target;
    target.joint_position.fill(i == 3 ? 999. : .5);
    client.get_command_interface()->buffer_command_target(target);
    CHECK(app.step());
    result.submitted.push_back(client.get_command_interface()->get_command().joint_position);
    const auto state = client.get_state_interface()->get_state();
    result.ros_position.push_back(state.measured_joint_position);
    result.ros_measured_torque.push_back(state.measured_torque);
    result.ros_external_torque.push_back(state.external_torque);
    if (!fill_queue) while (recorder.writer_step(writer)) {}
  }
  if (enabled && !fill_queue) {
    c::Recorder timed({2, 1000000000, 60000000000, 4096, 4096});
    CHECK(timed.start(2, now));
    auto timed_endpoint = endpoint; timed_endpoint.context = &timed;
    RecorderAdapter observer(2, timed_endpoint, clock_ns);
    const auto bytes = packet(5, 4);
    std::array<uint64_t, 1000> elapsed{};
    for (auto &ns : elapsed) {
      const auto start = StreamingCaptureSession::now_ns();
      track_allocations = true;
      observer.on_packet(inspect_fri_packet(bytes.data(), bytes.size()));
      observer.on_state(client.robotState());
      observer.on_submission(client.get_command_interface()->get_command().joint_position.data());
      track_allocations = false;
      ns = StreamingCaptureSession::now_ns() - start;
    }
    CHECK(allocations == 0 && timed.status().enqueued == 2000);
    std::sort(elapsed.begin(), elapsed.end());
    std::cout << "Offline observer parse/state/submission: median_ns=" << elapsed[500]
              << " p99_ns=" << elapsed[990] << " max_ns=" << elapsed.back()
              << "; C++ heap allocations=" << allocations << '\n';
  }
  recorder.stop(1);
  while (recorder.writer_step(writer)) {}
  result.sent = wire->sent;
  result.events = writer.events;
  result.status = recorder.status();
  return result;
}

void print(const char *label, const Joints &v) {
  std::cout << label << ": [";
  for (size_t j = 0; j < v.size(); ++j) std::cout << (j ? ", " : "") << v[j];
  std::cout << "]\n";
}
// Same actual driver/SDK fixture, now with the real streaming session and files.
std::vector<std::string> lifecycle(const std::filesystem::path &descriptor,
                                  const std::filesystem::path &root, bool enabled) {
  auto recording = enabled ? std::make_unique<FriRecording>(
      c::Config{42, 10000000000ULL, 60000000000, 24096, 4096}, root / "duration", descriptor, clock_ns)
      : nullptr;
  if (recording && !recording->start()) { CHECK(false); return {}; }
  auto adapter = recording ? recording->adapter() : nullptr;
  auto memory = std::make_unique<MemoryConnection>();
  auto *wire = memory.get();
  wire->inputs.push_back(packet(0, 2));
  for (unsigned i = 1; i <= 10004; ++i) wire->inputs.push_back(packet(i, 4));
  std::unique_ptr<KUKA::FRI::IConnection> connection;
  if (enabled) connection = std::make_unique<ObservedConnection>(std::move(memory), adapter);
  else connection = std::move(memory);
  lbr_fri_ros2::CommandGuardParameters guard;
  guard.min_positions.fill(-2.); guard.max_positions.fill(2.);
  guard.max_velocities.fill(10000.); guard.max_torques.fill(10000.);
  lbr_fri_ros2::AsyncClient client(KUKA::FRI::POSITION, .01, guard, "default", {}, {.04, .04}, true);
  if (enabled) client.set_recorder(adapter);
  KUKA::FRI::ClientApplication app(*connection, client);
  const auto base = StreamingCaptureSession::now_ns();
  for (unsigned i = 0; i <= 10004; ++i) {
    now = base + i * 1000000ULL;
    CHECK(app.step());
    if (enabled && i == 0) CHECK(!recording->status().capture.has_first);
    if (enabled && i && i % 128 == 0) {
      // Let the disk worker drain; fake time must not manufacture queue overflow.
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      const auto offered = recording->status().capture.enqueued;
      while (recording->status().capture.written < offered &&
             std::chrono::steady_clock::now() < deadline &&
             recording->status().capture.reason == c::Reason::None)
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      CHECK(recording->status().capture.written == offered);
    }
    if (enabled && i == 10001) {
      recording->wait();
      const auto status = recording->status();
      CHECK(status.capture.reason == c::Reason::Duration);
      CHECK(status.capture.enqueued == 20000 && status.capture.written == 20000);
      CHECK(status.capture.first_state_ns == base + 1000100);
      CHECK(status.capture.last_state_ns - status.capture.first_state_ns == 9999000000ULL);
      CHECK(status.joined && status.outcome_published && !status.runtime_failed);
      CHECK(std::filesystem::exists(root / "duration.capture.json"));
      recording.reset(); // Remaining callbacks retain a closed, valid endpoint.
    }
  }
  return wire->sent;
}
void terminal_cases(const std::filesystem::path &descriptor, const std::filesystem::path &root) {
  for (const auto reason : {c::Reason::NoData, c::Reason::SourceUnavailable, c::Reason::InvalidEvent, c::Reason::Duration}) {
    FriRecording recording({42, 5000000, reason == c::Reason::NoData ? 5000000ULL : 60000000000ULL, 64, 64},
                           root / std::to_string(unsigned(reason)), descriptor);
    if (!recording.start()) { CHECK(false); continue; }
    if (reason == c::Reason::SourceUnavailable) recording.source_stopped();
    if (reason == c::Reason::InvalidEvent) recording.adapter()->on_packet({});
    if (reason == c::Reason::Duration) {
      auto memory = std::make_unique<MemoryConnection>();
      memory->inputs = {packet(1, 4)};
      ObservedConnection connection(std::move(memory), recording.adapter());
      lbr_fri_ros2::CommandGuardParameters guard;
      guard.min_positions.fill(-2.); guard.max_positions.fill(2.);
      guard.max_velocities.fill(10000.); guard.max_torques.fill(10000.);
      lbr_fri_ros2::AsyncClient client(KUKA::FRI::POSITION, .01, guard, "default", {}, {.04, .04}, true);
      client.set_recorder(recording.adapter());
      KUKA::FRI::ClientApplication app(connection, client);
      CHECK(app.step());
      recording.wait(); // No next receive/callback/source-stop to drive expiry.
      CHECK(recording.status().capture.enqueued == 2);
    }
    recording.wait();
    CHECK(recording.status().capture.reason == reason);
    CHECK(recording.status().joined && recording.status().outcome_published);
  }
}
std::string file_bytes(const std::filesystem::path &path) {
  std::ifstream file(path, std::ios::binary);
  CHECK(file.good());
  return {std::istreambuf_iterator<char>{file}, {}};
}
// Same live source for capture, explicit stop, finalization and owner release.
// The boundary variant manually invokes observation after SDK decode so a stop
// can be placed deterministically between State and Submission admission.
std::vector<std::string> manual_lifecycle(const std::filesystem::path &descriptor,
                                        const std::filesystem::path &root,
                                        bool enabled, bool boundary = false) {
  const auto directory = root / (boundary ? "manual-boundary" : "manual");
  const auto outcome = std::filesystem::path(directory.string() + ".capture.json");
  auto recording = enabled ? std::make_unique<FriRecording>(
      c::Config{77, 1000000000, 60000000000, 128, 64}, directory, descriptor, clock_ns)
      : nullptr;
  if (recording && !recording->start()) { CHECK(false); return {}; }
  auto adapter = recording ? recording->adapter() : nullptr;
  auto memory = std::make_unique<MemoryConnection>();
  auto *wire = memory.get();
  wire->inputs.push_back(packet(0, 2));
  for (unsigned i = 1; i <= 9; ++i) wire->inputs.push_back(packet(i, 4));
  std::unique_ptr<KUKA::FRI::IConnection> connection;
  if (enabled) connection = std::make_unique<ObservedConnection>(std::move(memory), adapter);
  else connection = std::move(memory);
  lbr_fri_ros2::CommandGuardParameters guard;
  guard.min_positions.fill(-2.); guard.max_positions.fill(2.);
  guard.max_velocities.fill(10000.); guard.max_torques.fill(10000.);
  lbr_fri_ros2::AsyncClient client(KUKA::FRI::POSITION, .01, guard, "default", {}, {.04, .04}, true);
  if (enabled && !boundary) client.set_recorder(adapter);
  KUKA::FRI::ClientApplication app(*connection, client);
  const auto base = StreamingCaptureSession::now_ns();
  auto stop = [&] {
    const auto before = allocations;
    track_allocations = true;
    const bool accepted = recording->request_stop(77);
    track_allocations = false;
    CHECK(accepted && allocations == before);
  };
  for (unsigned i = 0; i <= 9; ++i) {
    now = base + i * 1000000ULL;
    if (enabled && i == 1) CHECK(!recording->request_stop(78));
    CHECK(app.step());
    if (boundary) {
      adapter->on_state(client.robotState());
      if (i == 3) stop();
      adapter->on_submission(client.get_command_interface()->get_command().joint_position.data());
    }
    if (enabled && i == 2) CHECK(recording->status().capture.reason == c::Reason::None);
    if (enabled && i == 3) {
      if (!boundary) stop();
      stop(); // Duplicate before finalization must be harmless.
    }
    if (enabled && i == 8) { // Five source cycles continued after the request.
      recording->wait();
      const auto status = recording->status();
      CHECK(status.capture.reason == c::Reason::Manual && status.capture.failure_flags == 0);
      CHECK(status.capture.has_first && status.capture.enqueued == (boundary ? 5u : 6u));
      CHECK(status.capture.written == status.capture.enqueued);
      CHECK(status.joined && status.outcome_published && !status.runtime_failed);
      CHECK(wire->next == 9 && wire->isOpen());
      const auto saved_outcome = file_bytes(outcome);
      const auto saved_signals = file_bytes(directory / "signals.mcap");
      stop(); // Late duplicate must not rewrite finalized output or counters.
      recording->wait();
      CHECK(recording->status().capture.enqueued == status.capture.enqueued);
      CHECK(recording->status().capture.written == status.capture.written);
      CHECK(file_bytes(outcome) == saved_outcome);
      CHECK(file_bytes(directory / "signals.mcap") == saved_signals);
      recording.reset(); // Cycle 9 still uses the retained closed endpoint.
    }
  }
  CHECK(wire->next == 10 && wire->isOpen() && wire->sent.size() == 5);
  return wire->sent;
}
void manual_stop_cases(const std::filesystem::path &descriptor,
                       const std::filesystem::path &root) {
  const auto commands = manual_lifecycle(descriptor, root, false);
  CHECK(manual_lifecycle(descriptor, root, true) == commands);
  CHECK(manual_lifecycle(descriptor, root, true, true) == commands);
  {
    FriRecording prepared({80, 1000000000, 60000000000, 64, 64},
                          root / "manual-prepared", descriptor);
    CHECK(!prepared.request_stop(81));
    CHECK(prepared.request_stop(80));
    CHECK(!prepared.start());
    prepared.wait();
    const auto status = prepared.status();
    CHECK(!status.started && !status.capture.has_first && !status.outcome_published);
  }
  CHECK(!std::filesystem::exists(root / "manual-prepared.capture.json"));
  for (const auto reason : {c::Reason::Manual, c::Reason::SourceUnavailable,
                            c::Reason::InvalidEvent}) {
    FriRecording armed({81, 1000000000, 60000000000, 64, 64},
                       root / ("manual-empty-" + std::to_string(unsigned(reason))), descriptor);
    if (!armed.start()) { CHECK(false); continue; }
    if (reason == c::Reason::SourceUnavailable) armed.source_stopped();
    if (reason == c::Reason::InvalidEvent) armed.adapter()->on_packet({});
    CHECK(armed.request_stop(81));
    armed.wait();
    const auto status = armed.status();
    CHECK(status.capture.reason == reason);
    CHECK(!status.capture.has_first && status.capture.enqueued == 0 && status.capture.written == 0);
    CHECK(status.joined && status.outcome_published && !status.runtime_failed);
    if (reason != c::Reason::Manual)
      CHECK(status.capture.failure_flags & (1u << unsigned(reason)));
  }
}
// A stable observer serves fresh, separately finalized recordings on one source.
// Workers may join before the FRI thread acknowledges detach; finish must wait.
void expect_detach_pending(FriRecordingManager &manager, uint64_t id) {
  const auto deadline = StreamingCaptureSession::now_ns() + 5000000000ULL;
  while (!manager.poll().session.joined && StreamingCaptureSession::now_ns() < deadline)
    std::this_thread::yield();
  CHECK(manager.status().session.joined && !manager.status().detached);
  CHECK(!manager.finish(id) && manager.status().owns_run);
}
void finish_manager(FriRecordingManager &manager, uint64_t id) {
  const auto deadline = StreamingCaptureSession::now_ns() + 5000000000ULL;
  while (!manager.finish(id) && StreamingCaptureSession::now_ns() < deadline) {
    manager.poll();
    std::this_thread::yield();
  }
  CHECK(manager.status().retired);
  CHECK(!manager.status().owns_run);
}
c::Config manager_config(uint64_t id) { return {id, 1000000000, 60000000000, 128, 64}; }
std::vector<std::string> manager_lifecycle(const std::filesystem::path &descriptor,
                                          const std::filesystem::path &root, bool enabled) {
  FriRecordingManager manager;
  const auto observer = manager.adapter();
  auto memory = std::make_unique<MemoryConnection>();
  auto *wire = memory.get();
  wire->inputs.push_back(packet(0, 2));
  for (unsigned i = 1; i < 12; ++i) wire->inputs.push_back(packet(i, 4));
  std::unique_ptr<KUKA::FRI::IConnection> connection;
  if (enabled) connection = std::make_unique<ObservedConnection>(std::move(memory), observer);
  else connection = std::move(memory);
  lbr_fri_ros2::CommandGuardParameters guard;
  guard.min_positions.fill(-2.); guard.max_positions.fill(2.);
  guard.max_velocities.fill(10000.); guard.max_torques.fill(10000.);
  lbr_fri_ros2::AsyncClient client(KUKA::FRI::POSITION, .01, guard, "default", {}, {.04, .04}, true);
  if (enabled) client.set_recorder(observer);
  KUKA::FRI::ClientApplication app(*connection, client);
  const auto base = StreamingCaptureSession::now_ns();
  for (unsigned i = 0; i < 12; ++i) {
    now = base + i * 1000000ULL;
    if (enabled && (i == 1 || i == 7)) {
      const uint64_t id = i == 1 ? 101 : 102;
      CHECK(manager.prepare(manager_config(id), root / (i == 1 ? "manager-a" : "manager-b"), descriptor, clock_ns));
      CHECK(manager.status().phase == RecordingManagerPhase::Prepared);
      CHECK(!manager.status().session.capture.has_first);
      CHECK(manager.start(id));
      CHECK(!manager.start(id));
      CHECK(manager.status().phase == RecordingManagerPhase::Armed);
      CHECK(!manager.status().attached && !manager.status().session.capture.has_first);
      CHECK(!manager.request_stop(id + 1));
      if (id == 102) CHECK(!manager.request_stop(101));
      CHECK(manager.adapter() == observer);
    }
    CHECK(app.step());
    if (enabled) {
      manager.poll();
      if (i == 1 || i == 7) {
        CHECK(manager.status().attached && manager.status().session.capture.has_first);
        CHECK(manager.status().phase == RecordingManagerPhase::Recording);
      }
      if (i == 3 || i == 9) {
        const auto id = i == 3 ? 101 : 102;
        CHECK(manager.request_stop(id));
        CHECK(manager.request_stop(id));
        CHECK(!manager.finish(id)); // A detached acknowledgement is still needed.
        CHECK(!manager.prepare(manager_config(id + 1), root / "manager-busy", descriptor));
      }
      if (i == 4 || i == 10) {
        const auto id = i == 4 ? 101 : 102;
        finish_manager(manager, id);
        const auto status = manager.status();
        CHECK(status.phase == RecordingManagerPhase::Finalized);
        CHECK(status.detached && status.retired && status.session.outcome_published);
        CHECK(status.session.capture.enqueued == 6 && status.session.capture.written == 6);
        CHECK(status.session.capture.reason == c::Reason::Manual);
        CHECK(manager.request_stop(id) && manager.finish(id));
        CHECK(!manager.prepare(manager_config(id), root / "manager-old-id", descriptor));
      }
    }
  }
  CHECK(wire->isOpen() && wire->next == 12);
  manager.producer_stopped(); // All SDK calls have returned; no future producer calls.
  return wire->sent;
}
void manager_edge_cases(const std::filesystem::path &descriptor,
                        const std::filesystem::path &root) {
  const auto baseline = manager_lifecycle(descriptor, root, false);
  CHECK(manager_lifecycle(descriptor, root, true) == baseline);
  {
    FriRecordingManager m;
    CHECK(!m.prepare(manager_config(0), root / "manager-zero", descriptor));
    CHECK(m.prepare(manager_config(110), root / "manager-prepared", descriptor));
    CHECK(m.request_stop(110));
    finish_manager(m, 110);
    CHECK(m.status().phase == RecordingManagerPhase::Cancelled);
    CHECK(!m.status().session.outcome_published);
    CHECK(m.prepare(manager_config(111), root / "manager-pending", descriptor));
    CHECK(m.start(111));
    CHECK(m.request_stop(111)); // Stop before attach has even been seen.
    m.poll();
    CHECK(!m.status().detached && !m.finish(111)); // Initial idle ack is insufficient.
    auto observer = m.adapter();
    observer->reset_packet(); // Attach ack, admission already closed.
    m.poll();
    CHECK(!m.status().detached);
    observer->reset_packet(); // Detach ack.
    m.poll();
    finish_manager(m, 111);
    CHECK(m.status().phase == RecordingManagerPhase::Failed); // Empty capture is not success.
    CHECK(!m.prepare(manager_config(112), root / "manager-after-empty", descriptor));
    m.producer_stopped();
  }
  {
    FriRecordingManager m;
    CHECK(m.prepare(manager_config(120), root / "manager-no-cycles", descriptor));
    CHECK(m.start(120));
    CHECK(m.request_stop(120));
    expect_detach_pending(m, 120); // Attachment never acknowledged: still pending.
    m.producer_stopped(); // Known producer quiescence substitutes for a missing ack.
    finish_manager(m, 120);
    CHECK(!m.prepare(manager_config(121), root / "manager-after-source-end", descriptor));
  }
  {
    FriRecordingManager m;
    CHECK(!m.prepare(manager_config(130), root, descriptor)); // Never reuse a directory.
    CHECK(m.status().phase == RecordingManagerPhase::Failed);
    CHECK(!m.status().owns_run);
  }
  for (unsigned fault = 0; fault < 3; ++fault) {
    FriRecordingManager m;
    const auto id = 140 + fault;
    auto cfg = manager_config(id);
    if (fault == 1) cfg.first_sample_timeout_ns = 1000000;
    CHECK(m.prepare(cfg, root / ("manager-fault-" + std::to_string(fault)), descriptor));
    CHECK(m.start(id));
    m.adapter()->reset_packet();
    if (fault == 0) {
      std::thread notify([observer = m.adapter()] { observer->source_stopped(); });
      notify.join();
    } else if (fault == 2) m.adapter()->on_packet({});
    const auto deadline = StreamingCaptureSession::now_ns() + 5000000000ULL;
    while (m.status().phase != RecordingManagerPhase::Stopping && StreamingCaptureSession::now_ns() < deadline) {
      m.poll();
      std::this_thread::yield();
    }
    m.adapter()->reset_packet();
    finish_manager(m, id);
    CHECK(m.status().phase == RecordingManagerPhase::Failed);
    CHECK(m.status().session.capture.reason == (fault == 0 ? c::Reason::SourceUnavailable :
          fault == 1 ? c::Reason::NoData : c::Reason::InvalidEvent));
    CHECK(!m.prepare(manager_config(id + 1), root / "manager-after-fault", descriptor));
    m.producer_stopped();
  }
}

// Duration, repeated runs, routed boundary stop, shutdown and failed finalization.
void manager_routed_cases(const std::filesystem::path &descriptor,
                          const std::filesystem::path &root) {
  using P = RecordingManagerPhase;
  { // Automatic duration, then ten Manual runs on one connection and one adapter.
    FriRecordingManager m;
    DriverRig rig(m.adapter(), 40);
    auto step = [&] { rig.step(); m.poll(); };
    step(); // Session-2 packet while idle: nothing is routed.
    CHECK(m.prepare({201, 5000000, 60000000000, 128, 64}, root / "manager-duration", descriptor));
    CHECK(m.start(201));
    step();
    CHECK(m.status().phase == P::Recording);
    const auto deadline = StreamingCaptureSession::now_ns() + 5000000000ULL;
    while (m.poll().phase != P::Stopping && StreamingCaptureSession::now_ns() < deadline)
      std::this_thread::yield();
    expect_detach_pending(m, 201); // Duration closed admission; no detach cycle yet.
    step();
    finish_manager(m, 201);
    auto s = m.status();
    CHECK(s.phase == P::Finalized && s.session.capture.reason == c::Reason::Duration);
    CHECK(s.session.capture.enqueued == 2 && s.session.capture.written == 2);
    for (uint64_t id = 202; id < 212; ++id) {
      CHECK(m.prepare(manager_config(id), root / ("manager-repeat-" + std::to_string(id)),
                      descriptor, clock_ns));
      CHECK(m.start(id));
      step(); step();
      CHECK(m.request_stop(id));
      if (id == 202) expect_detach_pending(m, id);
      step(); // Detach is acknowledged at this cycle's receive entry; nothing admitted.
      finish_manager(m, id);
      s = m.status();
      CHECK(s.phase == P::Finalized && s.run_id == id && !s.owns_run);
      CHECK(s.session.capture.reason == c::Reason::Manual);
      CHECK(s.session.capture.enqueued == 4 && s.session.capture.written == 4);
    }
    CHECK(rig.wire->next == 33 && rig.wire->isOpen());
    m.producer_stopped();
  }
  { // Stop between a routed State and its Submission; routed hooks allocate nothing.
    FriRecordingManager m;
    const auto observer = m.adapter();
    DriverRig rig(observer, 8, false); // Receive is observed; State/Submission driven here.
    size_t routed_allocations = 0;
    auto tracked = [&](auto &&call) {
      const auto before = allocations;
      track_allocations = true;
      call();
      track_allocations = false;
      routed_allocations += allocations - before;
    };
    const auto bytes = packet(5, 4);
    const auto info = inspect_fri_packet(bytes.data(), bytes.size());
    for (unsigned i = 0; i < 6; ++i) {
      if (i == 1) {
        CHECK(m.prepare(manager_config(220), root / "manager-boundary", descriptor, clock_ns));
        CHECK(m.start(220));
      }
      // Receive-path routing (cycle-entry latch and packet forwarding) is also
      // allocation-free; the following real receive resets this packet state.
      tracked([&] { observer->reset_packet(); observer->on_packet(info); });
      rig.step();
      tracked([&] { observer->on_state(rig.client->robotState()); });
      if (i == 3) CHECK(m.request_stop(220));
      tracked([&] {
        observer->on_submission(rig.client->get_command_interface()->get_command().joint_position.data());
      });
      m.poll();
    }
    CHECK(routed_allocations == 0);
    finish_manager(m, 220);
    const auto s = m.status();
    CHECK(s.phase == P::Finalized && s.session.capture.reason == c::Reason::Manual);
    CHECK(s.session.capture.enqueued == 5 && s.session.capture.written == 5);
    m.producer_stopped();
  }
  for (unsigned state = 0; state < 4; ++state) { // Prepared, Armed, Recording, Stopping.
    const auto id = 230 + state;
    const auto directory = root / ("manager-shutdown-" + std::to_string(state));
    {
      FriRecordingManager m;
      CHECK(m.prepare(manager_config(id), directory, descriptor, clock_ns));
      if (state >= 1) CHECK(m.start(id));
      if (state >= 2) {
        DriverRig rig(m.adapter(), 2);
        rig.step(); rig.step();
        CHECK(m.poll().phase == P::Recording);
        if (state == 3) { // Manual stop while the producer is live; detach left pending.
          CHECK(m.request_stop(id));
          CHECK(m.status().phase == P::Stopping && !m.status().detached);
        }
      } // Closing the rig ends the producer before the manager is destroyed.
    } // Destructor: producer_stopped(), then joins any finalizer; must not hang.
    const auto outcome = std::filesystem::path(directory.string() + ".capture.json");
    CHECK(std::filesystem::exists(outcome) == (state >= 1));
    if (state >= 2) CHECK(std::filesystem::exists(directory / "signals.mcap"));
    if (state >= 1) { // Recording closed by source end; Stopping keeps its first Manual cause.
      const auto reason = file_bytes(outcome).find(state == 3 ? "\"reason\":\"Manual\""
                                                               : "\"reason\":\"SourceUnavailable\"");
      CHECK(reason != std::string::npos);
    }
  }
  { // Complete Manual capture, no detach cycle, then confirmed producer end: still Finalized.
    FriRecordingManager m;
    CHECK(m.prepare(manager_config(250), root / "manager-quiesced-complete", descriptor, clock_ns));
    CHECK(m.start(250));
    {
      DriverRig rig(m.adapter(), 3);
      rig.step(); rig.step(); rig.step();
      CHECK(m.request_stop(250));
      expect_detach_pending(m, 250);
    } // Rig closure reports source loss after Manual was latched.
    m.producer_stopped();
    finish_manager(m, 250);
    const auto s = m.status();
    CHECK(s.phase == P::Finalized && s.session.capture.reason == c::Reason::Manual);
    CHECK(s.session.capture.enqueued == 4 && s.session.capture.written == 4);
    CHECK(!m.prepare(manager_config(251), root / "manager-after-quiesced", descriptor));
  }
  { // Failed outcome publication retains the run's evidence and blocks progression.
    const auto parent = root / "manager-readonly";
    std::filesystem::create_directories(parent);
    FriRecordingManager m;
    CHECK(m.prepare(manager_config(240), parent / "run", descriptor, clock_ns));
    CHECK(m.start(240));
    DriverRig rig(m.adapter(), 4);
    rig.step(); rig.step(); rig.step();
    std::filesystem::permissions(parent, std::filesystem::perms::owner_read |
                                         std::filesystem::perms::owner_exec);
    CHECK(m.request_stop(240));
    rig.step(); // Detach acknowledgement.
    finish_manager(m, 240);
    std::filesystem::permissions(parent, std::filesystem::perms::owner_all);
    const auto s = m.status();
    CHECK(s.phase == P::Failed && s.session.capture.has_first);
    CHECK(s.session.outcome_failed && !s.session.outcome_published);
    CHECK(std::filesystem::exists(parent / "run" / "signals.mcap"));
    CHECK(!m.prepare(manager_config(241), root / "manager-after-outcome-failure", descriptor));
    m.producer_stopped();
  }
}

void setup_cases(const std::filesystem::path &root) {
  unsetenv("LBR_RECORDING_DIRECTORY");
  unsetenv("LBR_RECORDING_DURATION_SECONDS");
  CHECK(!FriRecording::from_environment());
  for (const auto &path : {std::string{}, std::string("relative"), root.string()}) {
    setenv("LBR_RECORDING_DIRECTORY", path.c_str(), 1);
    bool rejected = false;
    try { FriRecording::from_environment(); } catch (const std::invalid_argument &) { rejected = true; }
    CHECK(rejected);
  }
  for (const char *value : {"", "0", "-1", "nan", "inf", "1e3", "10x", "1.",
                             "1.0000000001", "18446744074", "999999999999999999999999999"}) {
    setenv("LBR_RECORDING_DURATION_SECONDS", value, 1);
    setenv("LBR_RECORDING_DIRECTORY", (root / "invalid-duration").c_str(), 1);
    bool rejected = false;
    try { FriRecording::from_environment(); } catch (const std::invalid_argument &) { rejected = true; }
    CHECK(rejected && !std::filesystem::exists(root / "invalid-duration"));
  }
  for (const auto &entry : {std::pair<const char *, const char *>{"10", "10000000000"},
                            {"60.25", "60250000000"}, {".125", "125000000"}}) {
    const auto path = root / (std::string("duration-config-") + entry.first);
    setenv("LBR_RECORDING_DURATION_SECONDS", entry.first, 1);
    setenv("LBR_RECORDING_DIRECTORY", path.c_str(), 1);
    auto prepared = FriRecording::from_environment(); // No start and no socket.
    std::ifstream file(path / "build.txt");
    const std::string contents(std::istreambuf_iterator<char>{file}, {});
    CHECK(contents.find(std::string("duration_ns=") + entry.second + "\n") != std::string::npos);
    CHECK(contents.find("queue_capacity=4096\n") != std::string::npos);
  }
  unsetenv("LBR_RECORDING_DURATION_SECONDS");
  setenv("LBR_RECORDING_DIRECTORY", (root / "prepared").c_str(), 1);
  lbr_fri_ros2::CommandGuardParameters guard;
  guard.min_positions.fill(-2.); guard.max_positions.fill(2.);
  guard.max_velocities.fill(10000.); guard.max_torques.fill(10000.);
  auto client = std::make_shared<lbr_fri_ros2::AsyncClient>(KUKA::FRI::POSITION, .01, guard,
      "default", lbr_fri_ros2::StateGuardParameters{}, lbr_fri_ros2::StateInterfaceParameters{.04, .04}, true);
  { lbr_fri_ros2::App app(client); } // Construction/destruction only: never open a socket.
  CHECK(std::filesystem::exists(root / "prepared" / "build.txt"));
  CHECK(!std::filesystem::exists(root / "prepared.capture.json"));
  unsetenv("LBR_RECORDING_DIRECTORY");
}
int main(int argc, char **argv) {
  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_FATAL);
  const auto disabled = run(false), enabled = run(true), full = run(true, true);
  CHECK(disabled.sent.size() == 3); // receiveMultiplier=2, six callbacks.
  CHECK(enabled.sent == disabled.sent);
  CHECK(full.sent == disabled.sent);
  CHECK(enabled.submitted == disabled.submitted && full.submitted == disabled.submitted);
  CHECK(disabled.events.empty());
  CHECK(enabled.events.size() == 6); // three eligible states/submissions, no transition duplicates.
  CHECK(enabled.status.reason == c::Reason::SourceUnavailable);
  CHECK(full.status.reason == c::Reason::Overflow);
  CHECK(full.events.size() == 1 && full.events.front().kind == c::Kind::State);
  if (enabled.events.size() == 6) {
    for (unsigned i = 1; i <= 3; ++i) {
      const auto &state = enabled.events[(i - 1) * 2];
      const auto &submission = enabled.events[(i - 1) * 2 + 1];
      CHECK(state.kind == c::Kind::State && state.state_id == state.id);
      CHECK(state.position == values(.1 + .001 * i));
      CHECK(state.metadata.presence == 63 && state.metadata.controller_sec == 42);
      CHECK(state.metadata.controller_nsec == i * 1000000);
      CHECK(state.metadata.reported_sample_period_s == .001);
      CHECK(state.torque_presence == (i == 3 ? 2u : 7u));
      CHECK(state.tau_commanded_nm == values(-3. - i));
      if (i != 3) {
        CHECK(state.tau_measured_nm == values(10. + i));
        CHECK(state.tau_external_estimated_nm == values(.5 + i));
      } else {
        CHECK(state.tau_measured_nm == Joints{} && state.tau_external_estimated_nm == Joints{});
      }
      CHECK(submission.kind == c::Kind::Submission && submission.state_id == state.id);
      CHECK(submission.position == enabled.submitted[i]);
      CHECK(submission.host_monotonic_ns > state.host_monotonic_ns);
      CHECK(submission.torque_presence == 0 && submission.metadata.presence == 0);
    }
    const auto &s = enabled.events[2], &u = enabled.events[3];
    CHECK(s.position != enabled.ros_position[2]);
    CHECK(s.tau_measured_nm != enabled.ros_measured_torque[2]);
    CHECK(s.tau_external_estimated_nm != enabled.ros_external_torque[2]);
    CHECK(u.position[0] > .101 && u.position[0] < .5); // target filtered, not raw or requested q.
    CHECK(enabled.events[5].position == enabled.ros_position[3]); // guard neutralized target 999.
    std::cout << "Synthetic State id=" << s.id << " host_ns=" << s.host_monotonic_ns
              << "; Submission state_id=" << u.state_id << " host_ns=" << u.host_monotonic_ns << '\n';
    print("actual_q_rad", s.position); print("submitted_q_rad", u.position);
    print("measured_tau_Nm", s.tau_measured_nm); print("commanded_tau_Nm", s.tau_commanded_nm);
    print("estimated_external_tau_Nm", s.tau_external_estimated_nm);
  }
  if (argc == 3) {
    const std::filesystem::path root = argv[2];
    std::filesystem::remove_all(root); // Dedicated build-tree fixture output only.
    std::filesystem::create_directories(root);
    const auto commands = lifecycle(argv[1], root, false);
    CHECK(lifecycle(argv[1], root, true) == commands);
    terminal_cases(argv[1], root);
    manual_stop_cases(argv[1], root);
    manager_edge_cases(argv[1], root);
    manager_routed_cases(argv[1], root);
    setup_cases(root);
  } else CHECK(false);
  if (failures) { std::cerr << failures << " failed checks\n"; return 1; }
  std::cout << "Missing torques unavailable; all 3 outgoing messages identical with capture off/on/full.\n";
}
