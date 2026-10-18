#pragma once
#include "lbr_acquisition/offline_writer.hpp"
#include "lbr_acquisition/records.hpp"

namespace lbr_acquisition {
// Allocating, bounded OFFLINE conversion; never call from a capture hook.
// Preserves order/values. Rejects mixed runs, unsupported fields and malformed
// records with InvalidInput. The checked numeric run ID is not a wire field;
// persistent run identity belongs to the later bundle adapter.
std::vector<OfflineEvent>
encode_capture_batch(const std::vector<capture::Event> &events,
                     uint64_t expected_run_id);
OfflineEvent encode_capture_event(const capture::Event &event,
                                  uint64_t expected_run_id,
                                  uint64_t latest_state_id);
} // namespace lbr_acquisition
