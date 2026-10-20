#pragma once
#include "lbr_acquisition/records.hpp"
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace lbr_acquisition {
enum class FriPacketError : uint8_t {
  None, InvalidInput, Malformed, DuplicateField, MissingRequiredField,
  WrongMessageType, InvalidArrayLength
};

struct FriArrayInfo {
  bool present = false;
  std::size_t count = 0;
};

struct FriPacketInfo {
  FriPacketError error = FriPacketError::InvalidInput;
  FriArrayInfo measured_position{}, measured_torque{}, commanded_torque{}, external_torque{};
  // Uses capture::Presence. Values are read from the SDK only after its decode succeeds.
  uint32_t metadata_presence = 0;
  bool valid() const noexcept { return error == FriPacketError::None; }
};
static_assert(std::is_trivially_copyable_v<FriPacketInfo>);

// FRI SDK 1.15 MonitoringMessageDecoder::FRI_MONITOR_MSG_MAX_SIZE.
inline constexpr std::size_t fri_monitor_packet_max_size = 1500;

// Inspect only the FRI 1.15 LBR monitoring fields needed by the recorder.
// No allocation, retained pointers, value decoding, I/O, or SDK dependency.
// Accept packed/unpacked repeated doubles (including split packed segments).
// Reject duplicate inspected singular fields, malformed encodings and present
// joint arrays whose count is not seven. An error clears ALL availability.
// Unknown fields are skipped with checked wire lengths; their message contents
// and other SDK semantics (including checksums/enums) are NOT validated here.
// Success is not packet acceptance by the SDK, a fresh sensor value, or capture
// eligibility. The later adapter must require successful SDK decode as well.
FriPacketInfo inspect_fri_packet(const char *data, std::size_t size) noexcept;
} // namespace lbr_acquisition
