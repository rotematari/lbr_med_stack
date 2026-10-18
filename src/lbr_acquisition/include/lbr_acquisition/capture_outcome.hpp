#pragma once
#include "lbr_acquisition/offline_batch_writer.hpp"
#include "lbr_acquisition/streaming_capture_writer.hpp"

namespace lbr_acquisition {
// Offline-only, explicit post-drain publication. The caller must have stopped
// and joined all Recorder roles before obtaining status. The sidecar is a sibling
// of the run directory so pre-directory failures can still be recorded. This
// call neither changes Recorder status nor gives power-loss durability.
void write_capture_outcome(const OfflineBatchWriter &sink,
                           const capture::Status &status, FileOps &io);
void write_capture_outcome(const StreamingCaptureWriter &sink,
                           const capture::Status &status, FileOps &io);
} // namespace lbr_acquisition
