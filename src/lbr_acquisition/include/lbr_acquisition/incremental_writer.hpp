#pragma once
#include "lbr_acquisition/offline_writer.hpp"
#include "lbr_acquisition/capture_source.hpp"
#include "lbr_acquisition/capture_summary.hpp"
#include "lbr_acquisition/limits.hpp"
#include <memory>

namespace lbr_acquisition {
// Offline fixture format engine. Single writer owner; never used in capture hook.
// It creates a fresh directory and retains partial artifacts on failure.
class IncrementalFixtureWriter {
public:
  IncrementalFixtureWriter(std::filesystem::path directory,
                           std::filesystem::path descriptor, FileOps &io,
                           CaptureSource source = CaptureSource::Synthetic,
                           CaptureLimits limits = {}, bool latest_state_only = false);
  ~IncrementalFixtureWriter();
  IncrementalFixtureWriter(const IncrementalFixtureWriter &) = delete;
  IncrementalFixtureWriter &operator=(const IncrementalFixtureWriter &) = delete;
  void append(const OfflineEvent &event, bool close_chunk);
  void finish(const std::vector<OfflineEvent> &expected);
  void finish_capture(const CaptureSummary &expected);
  const std::filesystem::path &output_directory() const noexcept;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace lbr_acquisition
