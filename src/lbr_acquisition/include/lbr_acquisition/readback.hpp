#pragma once
#include "lbr_acquisition/offline_writer.hpp"
#include "lbr_acquisition/capture_source.hpp"
#include "lbr_acquisition/capture_summary.hpp"
#include "lbr_acquisition/limits.hpp"

namespace lbr_acquisition {
// Streaming full-file verification: at most one bounded MCAP record in RAM.
CaptureSource verify_capture_container(const std::filesystem::path &path,
                                      const CaptureSummary &expected,
                                      const std::string &descriptor,
                                      CaptureSource source,
                                      CaptureLimits limits);
// Independent, bounded linear reader. Throws IoError; never rewrites the
// source.
// PHD-FRI-RECORDER: returns the validated source observed in both channels.
CaptureSource verify_container(const std::filesystem::path &path,
                               const std::vector<OfflineEvent> &expected,
                               const std::string &descriptor,
                               CaptureSource source = CaptureSource::Synthetic);
} // namespace lbr_acquisition
