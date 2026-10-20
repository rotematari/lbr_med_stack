// PHD-FRI-RECORDER: project-owned public-SDK adapter; no control or storage work.
#include "lbr_acquisition/fri_adapter.hpp"
#include "lbr_acquisition/fri_recording_manager.hpp"
#include "friClientVersion.h"
#include "lbr_acquisition/streaming_capture_session.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string_view>
#include <limits>
#include "fri_build.hpp"

#if FRI_CLIENT_VERSION_MAJOR != 1 || FRI_CLIENT_VERSION_MINOR != 15
#error "PHD-FRI-RECORDER packet inspection is pinned to FRI SDK 1.15"
#endif

namespace lbr_acquisition {
namespace c = capture;
namespace {
uint32_t nonfinite_mask(const std::array<double, 7> &values) noexcept {
  uint32_t mask = 0;
  for (size_t j = 0; j < values.size(); ++j)
    if (!std::isfinite(values[j])) mask |= uint32_t{1} << j;
  return mask;
}
}
RecorderAdapter::RecorderAdapter(uint64_t id, CaptureEndpoint endpoint, Clock clock,
                                 std::shared_ptr<void> owner)
    : run_id_(id), endpoint_(endpoint), clock_(clock), owner_(std::move(owner)) {
  if (!id || !endpoint.context || !endpoint.offer || !endpoint.request_stop || !clock)
    throw std::invalid_argument("FRI recorder requires a run, endpoint and monotonic clock");
}
RecorderAdapter::RecorderAdapter(std::shared_ptr<detail::RecordingRoute> route)
    : run_id_(0), endpoint_{}, clock_(nullptr), route_(std::move(route)) {}
void RecorderAdapter::source_stopped() noexcept {
  if (route_) { route_->source_failed.store(true); return; }
  fail(c::Reason::SourceUnavailable);
}
void RecorderAdapter::reset_packet() noexcept {
  if (route_) {
    if (route_->closed.load()) return;
    const auto requested = route_->requested.load(std::memory_order_acquire);
    if (requested != routed_sequence_) {
      routed_target_ = route_->target;
      routed_sequence_ = requested;
      route_->acknowledged.store(requested, std::memory_order_release);
    }
    if (routed_target_ && !route_->source_failed.load()) routed_target_->reset_packet();
    return;
  }
  packet_pending_ = false;
  accepted_state_ = 0;
  packet_ = {};
}
void RecorderAdapter::fail(c::Reason reason) noexcept {
  accepted_state_ = 0;
  if (!failed_) {
    failed_ = true;
    endpoint_.request_stop(endpoint_.context, run_id_, reason);
  }
}
void RecorderAdapter::on_packet(const FriPacketInfo &packet) noexcept {
  if (route_) {
    if (!route_->closed.load() && !route_->source_failed.load() && routed_target_)
      routed_target_->on_packet(packet);
    return; // Never relatch routing here: receive/decode is already in progress.
  }
  reset_packet();
  if (failed_) return;
  if (!packet.valid()) {
    fail(c::Reason::InvalidEvent);
    return;
  }
  packet_ = packet;
  packet_pending_ = true;
}
void RecorderAdapter::on_state(const KUKA::FRI::LBRState &state) noexcept {
  if (route_) {
    if (!route_->closed.load() && !route_->source_failed.load() && routed_target_)
      routed_target_->on_state(state);
    return;
  }
  accepted_state_ = 0;
  if (failed_ || !packet_pending_) return;
  packet_pending_ = false; // Only a successful SDK decode/phase callback consumes this packet.
  c::Event event{};
  event.run_id = run_id_;
  event.id = event.state_id = ++state_sequence_;
  event.host_monotonic_ns = clock_();
  auto &meta = event.metadata;
  meta.presence = packet_.metadata_presence;
  if (meta.presence & c::ControllerTime) {
    meta.controller_sec = state.getTimestampSec();
    meta.controller_nsec = state.getTimestampNanoSec();
  }
  if (meta.presence & c::SamplePeriod) meta.reported_sample_period_s = state.getSampleTime();
  if (meta.presence & c::Session) meta.session_state = state.getSessionState();
  if (meta.presence & c::Quality) meta.connection_quality = state.getConnectionQuality();
  if (meta.presence & c::Control) meta.control_mode = state.getControlMode();
  if (meta.presence & c::Command) meta.client_command_mode = state.getClientCommandMode();
  constexpr uint32_t required = c::SamplePeriod | c::Session | c::Control | c::Command;
  const bool position_mode = (meta.presence & required) == required &&
      meta.session_state == KUKA::FRI::COMMANDING_ACTIVE &&
      meta.control_mode == KUKA::FRI::POSITION_CONTROL_MODE &&
      meta.client_command_mode == KUKA::FRI::POSITION;
  const bool valid_period = std::isfinite(meta.reported_sample_period_s) &&
                            meta.reported_sample_period_s >= .001;
  if (position_mode && !valid_period) {
    fail(c::Reason::SourceUnavailable);
    return;
  }
  const bool eligible = position_mode && valid_period && packet_.measured_position.present;
  if (eligible) {
    std::copy_n(state.getMeasuredJointPosition(), 7, event.position.begin());
    event.nonfinite_joint_mask = nonfinite_mask(event.position);
    if (packet_.measured_torque.present) {
      event.torque_presence |= c::MeasuredTorque;
      std::copy_n(state.getMeasuredTorque(), 7, event.tau_measured_nm.begin());
    }
    if (packet_.commanded_torque.present) {
      event.torque_presence |= c::CommandedTorque;
      std::copy_n(state.getCommandedTorque(), 7, event.tau_commanded_nm.begin());
    }
    if (packet_.external_torque.present) {
      event.torque_presence |= c::EstimatedExternalTorque;
      std::copy_n(state.getExternalTorque(), 7, event.tau_external_estimated_nm.begin());
    }
  }
  if (endpoint_.offer(endpoint_.context, event, eligible) == c::Offer::Accepted)
    accepted_state_ = event.id;
}
void RecorderAdapter::on_submission(const double *positions) noexcept {
  if (route_) {
    if (!route_->closed.load() && !route_->source_failed.load() && routed_target_)
      routed_target_->on_submission(positions);
    return;
  }
  if (failed_ || !accepted_state_) return;
  if (!positions) { fail(c::Reason::InvalidEvent); return; }
  c::Event event{};
  event.kind = c::Kind::Submission;
  event.run_id = run_id_;
  event.id = ++submission_sequence_;
  event.state_id = accepted_state_;
  event.host_monotonic_ns = clock_();
  std::copy_n(positions, 7, event.position.begin());
  event.nonfinite_joint_mask = nonfinite_mask(event.position);
  endpoint_.offer(endpoint_.context, event, true);
}
ObservedConnection::ObservedConnection(std::unique_ptr<KUKA::FRI::IConnection> connection,
                                       std::shared_ptr<RecorderAdapter> adapter)
    : connection_(std::move(connection)), adapter_(std::move(adapter)) {
  if (!connection_ || !adapter_)
    throw std::invalid_argument("ObservedConnection requires a connection and recorder adapter");
}
bool ObservedConnection::open(int port, const char *host) {
  const bool result = connection_->open(port, host);
  if (!result) adapter_->source_stopped();
  return result;
}
void ObservedConnection::close() { adapter_->source_stopped(); connection_->close(); }
bool ObservedConnection::isOpen() const { return connection_->isOpen(); }
int ObservedConnection::receive(char *buffer, int max_size) {
  adapter_->reset_packet(); // A failed receive/decode must never reuse an earlier association.
  const int size = connection_->receive(buffer, max_size);
  if (size > 0) {
    adapter_->on_packet(size <= max_size ? inspect_fri_packet(buffer, static_cast<size_t>(size))
                                        : FriPacketInfo{});
  } else adapter_->source_stopped();
  return size;
}
bool ObservedConnection::send(const char *buffer, int size) {
  const bool result = connection_->send(buffer, size);
  if (!result) adapter_->source_stopped();
  return result;
}

namespace {
// Storage stays alive through the adapter's shared owner. Only offers enter the
// session through this gate; stop notifications publish atomics, never join/I/O.
struct LiveEndpoint {
  static constexpr unsigned closed = 1, active = 2;
  const c::Config config;
  FileOps io;
  StreamingCaptureSession session;
  std::atomic<unsigned> gate{closed};
  std::atomic<c::Reason> requested{c::Reason::None};
  LiveEndpoint(c::Config cfg, const std::filesystem::path &directory,
               const std::filesystem::path &descriptor)
      : config(cfg), session(cfg, directory, descriptor, io,
          std::chrono::milliseconds(1), std::chrono::seconds(5), nullptr, 32, CaptureSource::FriSdk) {}
  static c::Offer offer(void *p, const c::Event &event, bool eligible) noexcept {
    auto &self = *static_cast<LiveEndpoint *>(p);
    unsigned expected = 0;
    if (!self.gate.compare_exchange_strong(expected, active)) return c::Offer::Inactive;
    const auto result = self.session.offer(event, eligible);
    self.gate.fetch_and(~active);
    return result;
  }
  static bool stop(void *p, uint64_t id, c::Reason reason) noexcept {
    auto &self = *static_cast<LiveEndpoint *>(p);
    if (id != self.config.run_id || reason == c::Reason::None) return false;
    self.gate.fetch_or(closed);
    auto expected = c::Reason::None;
    self.requested.compare_exchange_strong(expected, reason);
    return true;
  }
  void close_admission() noexcept {
    gate.fetch_or(closed);
    while (gate.load() & active) std::this_thread::yield(); // Finalizer/owner only.
  }
};
std::filesystem::path checked_directory(std::filesystem::path directory) {
  if (directory.empty() || !directory.is_absolute() || std::filesystem::exists(directory))
    throw std::invalid_argument("LBR_RECORDING_DIRECTORY must be a new absolute directory");
  return directory;
}
uint64_t duration_from_environment() {
  const char *raw = std::getenv("LBR_RECORDING_DURATION_SECONDS");
  if (!raw) return 1000000000ULL;
  const std::string_view text(raw);
  uint64_t seconds = 0, fraction = 0;
  unsigned places = 0;
  bool decimal = false, digit = false;
  const auto invalid = [] {
    throw std::invalid_argument("LBR_RECORDING_DURATION_SECONDS must be positive decimal seconds, at most 9 fractional digits, within uint64 nanoseconds");
  };
  for (char ch : text) {
    if (ch == '.' && !decimal) { decimal = true; continue; }
    if (ch < '0' || ch > '9') invalid();
    digit = true;
    const auto value = static_cast<unsigned>(ch - '0');
    if (decimal) {
      if (++places > 9) invalid();
      fraction = fraction * 10 + value;
    } else {
      if (seconds > (UINT64_MAX - value) / 10) invalid();
      seconds = seconds * 10 + value;
    }
  }
  if (!digit || (decimal && !places)) invalid();
  for (; places < 9; ++places) fraction *= 10;
  if (seconds > (UINT64_MAX - fraction) / 1000000000ULL) invalid();
  const auto ns = seconds * 1000000000ULL + fraction;
  if (!ns || ns > UINT64_MAX - StreamingCaptureSession::now_ns()) invalid();
  return ns;
}
}
struct FriRecording::Impl {
  std::shared_ptr<LiveEndpoint> endpoint;
  std::shared_ptr<RecorderAdapter> adapter;
  std::thread finalizer;
  bool started = false;
  Impl(c::Config config, const std::filesystem::path &directory,
       const std::filesystem::path &descriptor, RecorderAdapter::Clock clock)
      : endpoint(std::make_shared<LiveEndpoint>(config, checked_directory(directory), descriptor)),
        adapter(std::make_shared<RecorderAdapter>(config.run_id,
            CaptureEndpoint{endpoint.get(), LiveEndpoint::offer, LiveEndpoint::stop},
            clock ? clock : StreamingCaptureSession::now_ns, endpoint)) {}
  void finalize() noexcept {
    for (;;) {
      const auto requested = endpoint->requested.load();
      const auto status = endpoint->session.status();
      if (requested != c::Reason::None || status.capture.reason != c::Reason::None ||
          status.runtime_failed) {
        endpoint->close_admission();
        if (requested != c::Reason::None)
          endpoint->session.request_stop(endpoint->config.run_id, requested);
        endpoint->session.join();
        const auto final = endpoint->session.status();
        // Process-level diagnostics only, never on the FRI producer thread.
        std::cerr << "PHD-FRI-RECORDER: finalized; reason=" << unsigned(final.capture.reason)
                  << " written=" << final.capture.written
                  << " outcome_published=" << final.outcome_published << '\n';
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
};
FriRecording::FriRecording(c::Config config, std::filesystem::path directory,
                           std::filesystem::path descriptor, RecorderAdapter::Clock clock)
    : impl_(std::make_unique<Impl>(config, directory, descriptor, clock)) {}
FriRecording::~FriRecording() {
  LiveEndpoint::stop(impl_->endpoint.get(), impl_->endpoint->config.run_id, c::Reason::Manual);
  wait();
}
std::shared_ptr<FriRecording> FriRecording::from_environment() {
  const char *directory = std::getenv("LBR_RECORDING_DIRECTORY");
  if (!directory) return {};
  const auto duration = duration_from_environment();
  // FRI's 1 ms minimum cycle: one State and one Submission, plus a fixed
  // backlog allowance. This admission budget is independent of queue memory.
  const auto cycles = duration / 1000000 + (duration % 1000000 != 0);
  const c::Config config{StreamingCaptureSession::now_ns(), duration,
                         60000000000, cycles * 2 + 4096, 4096};
  return prepare(config, directory, LBR_BUILD_DESCRIPTOR);
}
std::shared_ptr<FriRecording> FriRecording::prepare(c::Config config,
    const std::filesystem::path &directory, const std::filesystem::path &descriptor,
    RecorderAdapter::Clock clock) {
  const auto limits = capture_limits_for(config.max_events);
  auto recording = std::make_shared<FriRecording>(config, directory, descriptor, clock);
  std::ofstream provenance;
  provenance.exceptions(std::ios::failbit | std::ios::badbit);
  provenance.open(std::filesystem::path(directory) / "build.txt");
  provenance << fri_build_provenance << "run_id=" << config.run_id
             << "\nduration_ns=" << config.duration_ns
             << "\nfirst_sample_timeout_ns=" << config.first_sample_timeout_ns
             << "\nmax_events=" << config.max_events
             << "\nfile_limit_bytes=" << limits.file_bytes
             << "\nstorage_event_limit=" << limits.events
             << "\nqueue_capacity=" << config.queue_capacity
             << "\npoll_ns=1000000\nstall_ns=5000000000"
                "\nverification=bounded-record-scan-and-sha256-chain-v1\n";
  provenance.close();
  return recording;
}
std::shared_ptr<RecorderAdapter> FriRecording::adapter() const { return impl_->adapter; }
bool FriRecording::start() {
  if (impl_->started || impl_->endpoint->requested.load() != c::Reason::None) return false;
  impl_->started = true;
  if (!impl_->endpoint->session.start()) return false;
  impl_->endpoint->gate.store(0);
  try { impl_->finalizer = std::thread([p = impl_.get()] { p->finalize(); }); }
  catch (...) {
    impl_->endpoint->close_admission();
    impl_->endpoint->session.request_stop(impl_->endpoint->config.run_id, c::Reason::Manual);
    impl_->endpoint->session.join();
    return false;
  }
  return true;
}
bool FriRecording::request_stop(uint64_t run_id) noexcept {
  return LiveEndpoint::stop(impl_->endpoint.get(), run_id, c::Reason::Manual);
}
void FriRecording::source_stopped() noexcept {
  LiveEndpoint::stop(impl_->endpoint.get(), impl_->endpoint->config.run_id, c::Reason::SourceUnavailable);
}
void FriRecording::wait() { if (impl_->finalizer.joinable()) impl_->finalizer.join(); }
StreamingSessionStatus FriRecording::status() const noexcept { return impl_->endpoint->session.status(); }
} // namespace lbr_acquisition
