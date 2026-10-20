// PHD-FRI-RECORDER: project-owned observation of public FRI 1.15 state/submissions.
#pragma once
#include "lbr_acquisition/fri_packet.hpp"
#include "lbr_acquisition/recorder.hpp"
#include "friConnectionIf.h"
#include "friLBRState.h"
#include <memory>
#include <filesystem>

namespace lbr_acquisition {
namespace detail { struct RecordingRoute; }
class FriRecordingManager;
struct CaptureEndpoint {
  void *context;
  capture::Offer (*offer)(void *, const capture::Event &, bool) noexcept;
  bool (*request_stop)(void *, uint64_t, capture::Reason) noexcept;
};

// One FRI-thread caller. Install before callbacks start; endpoint/clock must
// remain valid until all callers stop. Session admission/finalization is owned
// externally. These hooks neither own a worker nor issue robot commands.
class RecorderAdapter {
public:
  using Clock = uint64_t (*)() noexcept;
  RecorderAdapter(uint64_t run_id, CaptureEndpoint, Clock, std::shared_ptr<void> owner = {});
  void reset_packet() noexcept;
  void on_packet(const FriPacketInfo &) noexcept;
  void on_state(const KUKA::FRI::LBRState &) noexcept;
  void on_submission(const double *positions) noexcept;
  void source_stopped() noexcept;
private:
  friend class FriRecordingManager;
  explicit RecorderAdapter(std::shared_ptr<detail::RecordingRoute>);
  const uint64_t run_id_;
  const CaptureEndpoint endpoint_;
  const Clock clock_;
  const std::shared_ptr<void> owner_; // Retain endpoint storage even after its run owner closes.
  FriPacketInfo packet_{};
  bool packet_pending_ = false, failed_ = false;
  uint64_t state_sequence_ = 0, submission_sequence_ = 0, accepted_state_ = 0;
  const std::shared_ptr<detail::RecordingRoute> route_;
  RecorderAdapter *routed_target_ = nullptr; // FRI-thread only; manager pins its lifetime.
  uint64_t routed_sequence_ = 0;
  void fail(capture::Reason) noexcept;
};

struct StreamingSessionStatus;
// Setup/start/wait/destruction belong to the non-FRI owner thread. The finalizer
// closes admission and drains independently, so control need not stop at 1 s.
class FriRecording {
public:
  FriRecording(capture::Config, std::filesystem::path directory,
               std::filesystem::path descriptor,
               RecorderAdapter::Clock clock = nullptr);
  ~FriRecording();
  FriRecording(const FriRecording &) = delete;
  FriRecording &operator=(const FriRecording &) = delete;
  static std::shared_ptr<FriRecording> from_environment();
  static std::shared_ptr<FriRecording> prepare(capture::Config,
      const std::filesystem::path &directory, const std::filesystem::path &descriptor,
      RecorderAdapter::Clock clock = nullptr);
  std::shared_ptr<RecorderAdapter> adapter() const;
  bool start();
  // Nonblocking Manual stop; false only for a different run ID. True means
  // recognized, not finalized. Repeats preserve the existing terminal cause.
  // May overlap running callbacks/finalization, but never start() or destruction.
  // Before start, complete this call first to cancel the prepared run.
  bool request_stop(uint64_t run_id) noexcept;
  void source_stopped() noexcept;
  void wait();
  StreamingSessionStatus status() const noexcept;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Delegates the same bytes/results through the SDK's public connection seam.
// This wrapper does not create a connection or replace the SDK decoder.
class ObservedConnection final : public KUKA::FRI::IConnection {
public:
  ObservedConnection(std::unique_ptr<KUKA::FRI::IConnection>, std::shared_ptr<RecorderAdapter>);
  bool open(int port, const char *remote_host) override;
  void close() override;
  bool isOpen() const override;
  int receive(char *buffer, int max_size) override;
  bool send(const char *buffer, int size) override;
private:
  std::unique_ptr<KUKA::FRI::IConnection> connection_;
  std::shared_ptr<RecorderAdapter> adapter_;
};
} // namespace lbr_acquisition
