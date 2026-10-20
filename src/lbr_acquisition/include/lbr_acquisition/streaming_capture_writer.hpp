#pragma once
#include "lbr_acquisition/incremental_writer.hpp"
#include "lbr_acquisition/offline_batch_writer.hpp"
#include <memory>

namespace lbr_acquisition {
// Offline-only, serial writer-side access. No operation runs in the producer.
class StreamingCaptureWriter final : public capture::Writer {
public:
  using State = OfflineBatchWriter::State;
  StreamingCaptureWriter(uint64_t run_id, std::filesystem::path directory,
                         std::filesystem::path descriptor, FileOps &io,
                         CaptureSource source = CaptureSource::Synthetic,
                         CaptureLimits limits = {});
  ~StreamingCaptureWriter() override = default;
  StreamingCaptureWriter(const StreamingCaptureWriter &) = delete;
  StreamingCaptureWriter &operator=(const StreamingCaptureWriter &) = delete;
  void write(const capture::Event &) override;
  void finish() override;
  State state() const noexcept { return state_; }
  uint64_t run_id() const noexcept { return run_id_; }
  CaptureSource source() const noexcept { return source_; }
  const std::filesystem::path &output_directory() const noexcept { return directory_; }
  const CaptureSummary &summary() const noexcept { return summary_; }
  CaptureLimits limits() const noexcept { return limits_; }
private:
  uint64_t run_id_, latest_state_ = 0;
  const CaptureSource source_;
  std::filesystem::path directory_;
  const CaptureLimits limits_;
  CaptureSummary summary_;
  IncrementalFixtureWriter writer_;
  State state_ = State::Accepting;
};
} // namespace lbr_acquisition
