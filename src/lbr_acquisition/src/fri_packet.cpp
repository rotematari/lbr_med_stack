#include "lbr_acquisition/fri_packet.hpp"
#include <limits>

namespace lbr_acquisition {
namespace {
// Tags and required fields below follow the unmodified FRI SDK 1.15
// src/protobuf_gen/FRIMessages.pb.{h,c}. Message ID: include/friLBRState.h.
// Deliberately independent of nanopb's private message/buffer layout.
struct Bytes {
  const char *data;
  std::size_t size;
};
struct Field {
  uint32_t tag = 0;
  uint8_t wire = 0;
  uint64_t integer = 0;
  Bytes payload{nullptr, 0};
};

class Reader {
public:
  explicit Reader(Bytes bytes) noexcept : bytes_(bytes) {}
  bool empty() const noexcept { return offset_ == bytes_.size; }
  bool next(Field &field) noexcept {
    uint64_t key;
    if (!varint(key) || key > std::numeric_limits<uint32_t>::max() || (key >> 3) == 0)
      return false;
    field = {};
    field.tag = static_cast<uint32_t>(key >> 3);
    field.wire = static_cast<uint8_t>(key & 7);
    uint64_t length;
    switch (field.wire) {
    case 0: return varint(field.integer);
    case 1: return take(8, field.payload);
    case 2: return varint(length) && take(length, field.payload);
    case 5: return take(4, field.payload);
    default: return false; // Groups are outside this recorder's supported subset.
    }
  }

private:
  bool varint(uint64_t &value) noexcept {
    value = 0;
    for (unsigned i = 0; i < 10; ++i) {
      if (empty()) return false;
      const auto b = static_cast<unsigned char>(bytes_.data[offset_++]);
      if (i == 9 && b > 1) return false; // Includes overflow and unterminated varints.
      value |= uint64_t(b & 127) << (7 * i);
      if (!(b & 128)) return true;
    }
    return false;
  }
  bool take(uint64_t length, Bytes &part) noexcept {
    if (length > bytes_.size - offset_) return false;
    const auto n = static_cast<std::size_t>(length);
    part = {bytes_.data + offset_, n};
    offset_ += n;
    return true;
  }
  Bytes bytes_;
  std::size_t offset_ = 0;
};

enum class Message { Root, Header, Monitor, Connection, Robot, Ipo, Timestamp };

class Inspector {
public:
  FriPacketInfo run(Bytes bytes) noexcept {
    result_.error = FriPacketError::None;
    if (!message(bytes, Message::Root)) {
      const auto error = result_.error;
      result_ = {}; // Never return partial availability on error.
      result_.error = error;
    }
    return result_;
  }

private:
  bool fail(FriPacketError error) noexcept { result_.error = error; return false; }
  bool once(uint32_t &seen, const Field &field, uint8_t wire) noexcept {
    // Called only for the inspected tags (1..15).
    const uint32_t bit = uint32_t(1) << field.tag;
    if (seen & bit) return fail(FriPacketError::DuplicateField);
    if (field.wire != wire) return fail(FriPacketError::Malformed);
    seen |= bit;
    return true;
  }
  bool scalar(uint32_t &seen, const Field &field) noexcept {
    return once(seen, field, 0) &&
           (field.integer <= std::numeric_limits<uint32_t>::max() || fail(FriPacketError::Malformed));
  }
  bool child(uint32_t &seen, const Field &field, Message type) noexcept {
    return once(seen, field, 2) && message(field.payload, type);
  }
  bool array(uint32_t &seen, const Field &field, FriArrayInfo &array_info) noexcept {
    if (!once(seen, field, 2)) return false;
    Reader reader(field.payload);
    std::size_t count = 0;
    while (!reader.empty()) {
      Field value;
      if (!reader.next(value)) return fail(FriPacketError::Malformed);
      if (value.tag != 1) continue;
      if (value.wire == 1) ++count;
      else if (value.wire == 2 && value.payload.size % 8 == 0) count += value.payload.size / 8;
      else return fail(FriPacketError::Malformed);
      if (count > 7) return fail(FriPacketError::InvalidArrayLength);
    }
    if (count != 7) return fail(FriPacketError::InvalidArrayLength);
    array_info = {true, count};
    return true;
  }
  bool metadata(uint32_t &seen, const Field &field, uint32_t presence) noexcept {
    if (!scalar(seen, field)) return false;
    result_.metadata_presence |= presence;
    return true;
  }
  bool tracking(uint32_t &seen, const Field &field) noexcept {
    // A double (fixed64); its value, including 0 or nonfinite, is read from the SDK later.
    if (!once(seen, field, 1)) return false;
    result_.metadata_presence |= capture::Tracking;
    return true;
  }
  bool message(Bytes bytes, Message type) noexcept {
    // Only fixed schema edges recurse: Root -> Monitor -> Timestamp (depth 3).
    // Unknown length-delimited fields are opaque and never recurse.
    Reader reader(bytes);
    uint32_t seen = 0;
    uint32_t message_id = 0;
    while (!reader.empty()) {
      Field f;
      if (!reader.next(f)) return fail(FriPacketError::Malformed);
      switch (type) {
      case Message::Root:
        switch (f.tag) {
        case 1: if (!child(seen, f, Message::Header)) return false; break;
        case 2: if (!child(seen, f, Message::Robot)) return false; break;
        case 3: if (!child(seen, f, Message::Monitor)) return false; break;
        case 4: if (!child(seen, f, Message::Connection)) return false; break;
        case 5: if (!child(seen, f, Message::Ipo)) return false; break;
        default: break;
        }
        break;
      case Message::Header:
        if (f.tag >= 1 && f.tag <= 3) {
          if (!scalar(seen, f)) return false;
          if (f.tag == 1) message_id = static_cast<uint32_t>(f.integer);
        }
        break;
      case Message::Monitor:
        switch (f.tag) {
        case 1: if (!array(seen, f, result_.measured_position)) return false; break;
        case 2: if (!array(seen, f, result_.measured_torque)) return false; break;
        case 3: if (!array(seen, f, result_.commanded_position)) return false; break;
        case 4: if (!array(seen, f, result_.commanded_torque)) return false; break;
        case 5: if (!array(seen, f, result_.external_torque)) return false; break;
        case 15: if (!child(seen, f, Message::Timestamp)) return false; break;
        default: break;
        }
        break;
      case Message::Connection:
        switch (f.tag) {
        case 1: if (!metadata(seen, f, capture::Session)) return false; break;
        case 2: if (!metadata(seen, f, capture::Quality)) return false; break;
        case 3: if (!metadata(seen, f, capture::SamplePeriod)) return false; break;
        default: break;
        }
        break;
      case Message::Robot:
        if (f.tag == 2 && !metadata(seen, f, capture::Safety)) return false;
        if (f.tag == 7 && !metadata(seen, f, capture::Control)) return false;
        break;
      case Message::Ipo:
        if (f.tag == 10 && !metadata(seen, f, capture::Command)) return false;
        if (f.tag == 12 && !tracking(seen, f)) return false;
        break;
      case Message::Timestamp:
        if ((f.tag == 1 || f.tag == 2) && !scalar(seen, f)) return false;
        break;
      }
    }
    const uint32_t required = type == Message::Root ? 2u :
                              type == Message::Header ? 14u :
                              type == Message::Connection || type == Message::Timestamp ? 6u : 0u;
    if ((seen & required) != required) return fail(FriPacketError::MissingRequiredField);
    if (type == Message::Header && message_id != 0x245142)
      return fail(FriPacketError::WrongMessageType);
    if (type == Message::Timestamp) result_.metadata_presence |= capture::ControllerTime;
    return true;
  }
  FriPacketInfo result_{};
};
} // namespace

FriPacketInfo inspect_fri_packet(const char *data, std::size_t size) noexcept {
  if (!data || size == 0 || size > fri_monitor_packet_max_size) return {};
  return Inspector{}.run({data, size});
}
} // namespace lbr_acquisition
