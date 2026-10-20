// PHD-FRI-RECORDER: constant-size integrity evidence for acknowledged records.
#pragma once
#include "lbr_acquisition/offline_writer.hpp"
#include <array>
#include <string_view>

namespace lbr_acquisition {
struct CaptureSummary {
  uint64_t events = 0, states = 0, first_state_ns = 0, last_state_ns = 0;
  std::array<unsigned char, 32> digest{};
  // Hash chain: previous digest, record kind, LE host time, LE payload length,
  // exact protobuf payload. Ordering, type, values and timestamps are covered.
  void observe(bool state, uint64_t host_ns, std::string_view payload);
  void observe(const OfflineEvent &event);
  bool operator==(const CaptureSummary &other) const noexcept;
};
} // namespace lbr_acquisition
