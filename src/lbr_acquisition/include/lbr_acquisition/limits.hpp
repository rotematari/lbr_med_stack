// PHD-FRI-RECORDER: per-run storage budgets, independent of the fixed live queue.
#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace lbr_acquisition {
struct CaptureLimits {
  uint64_t events = 10000;
  uint64_t file_bytes = 16 * 1024 * 1024;
};
inline CaptureLimits checked_limits(CaptureLimits limits) {
  constexpr auto maximum = static_cast<uint64_t>(std::numeric_limits<std::ptrdiff_t>::max());
  if (!limits.events || !limits.file_bytes || limits.events > maximum ||
      limits.file_bytes > maximum)
    throw std::invalid_argument("capture budgets must be positive and representable");
  return limits;
}
inline CaptureLimits capture_limits_for(uint64_t events) {
  // At most 4096 payload bytes plus MCAP framing per event; leave ample margin.
  // This is a byte ceiling, not a RAM allocation or disk preallocation.
  constexpr auto maximum = static_cast<uint64_t>(std::numeric_limits<std::ptrdiff_t>::max());
  if (!events || events > (maximum - 65536) / 8192)
    throw std::invalid_argument("capture event budget exceeds representable file size");
  return {std::max<uint64_t>(10000, events),
          std::max<uint64_t>(16 * 1024 * 1024, events > 10000 ? events * 8192 + 65536 : 0)};
}
} // namespace lbr_acquisition
