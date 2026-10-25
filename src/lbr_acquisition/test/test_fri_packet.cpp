#include "lbr_acquisition/fri_packet.hpp"
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <new>
#include <string>

// Fail the test if the inspector starts using C++ heap allocation.
static bool deny_allocation = false;
void *operator new(std::size_t n) {
  if (deny_allocation) std::abort();
  if (void *p = std::malloc(n ? n : 1)) return p;
  throw std::bad_alloc();
}
void *operator new[](std::size_t n) { return ::operator new(n); }
void operator delete(void *p) noexcept { std::free(p); }
void operator delete[](void *p) noexcept { std::free(p); }
void operator delete(void *p, std::size_t) noexcept { std::free(p); }
void operator delete[](void *p, std::size_t) noexcept { std::free(p); }

using namespace lbr_acquisition;
namespace c = lbr_acquisition::capture;
int failures = 0;
#define CHECK(condition) do { if (!(condition)) { \
  std::cerr << __LINE__ << ": " #condition "\n"; ++failures; } } while (false)

// Test-only wire encoder: literal tags independently checked against SDK 1.15
// FRIMessages.pb.h/.c, not imported from the inspector's implementation.
std::string varint(uint64_t v) {
  std::string s;
  do { const auto byte = v & 127; v >>= 7; s += static_cast<char>(byte | (v ? 128 : 0)); } while (v);
  return s;
}
std::string integer(uint32_t tag, uint64_t v) { return varint(uint64_t(tag) << 3) + varint(v); }
std::string message(uint32_t tag, const std::string &s) {
  return varint((uint64_t(tag) << 3) | 2) + varint(s.size()) + s;
}
std::string doubles(std::size_t count, double value = 0.) {
  std::string s;
  uint64_t bits;
  static_assert(sizeof(value) == sizeof(bits));
  std::memcpy(&bits, &value, sizeof(bits));
  for (std::size_t i = 0; i < count; ++i)
    for (unsigned b = 0; b < 8; ++b) s += static_cast<char>(bits >> (b * 8));
  return s;
}
std::string joints(std::size_t count = 7, double value = 0.) { return message(1, doubles(count, value)); }
std::string fixed64(uint32_t tag, double value) { return varint((uint64_t(tag) << 3) | 1) + doubles(1, value); }
std::string header() { return message(1, integer(1, 0x245142) + integer(2, 1) + integer(3, 0)); }
std::string packet(const std::string &monitor) { return header() + message(3, monitor); }
std::string full_packet() {
  return packet(message(1, joints(7, .125)) + message(2, joints(7, 11.)) +
                message(3, joints(7, .375)) + message(4, joints(7, -3.)) +
                message(5, joints(7, .25)) + message(15, integer(1, 0) + integer(2, 0))) +
         message(4, integer(1, 4) + integer(2, 3) + integer(3, 1)) +
         message(2, integer(2, 0) + integer(7, 0)) + message(5, integer(10, 1) + fixed64(12, 1.));
}
FriPacketInfo inspect(const std::string &s) {
  const auto original = s;
  deny_allocation = true;
  const auto result = inspect_fri_packet(s.data(), s.size());
  deny_allocation = false;
  CHECK(s == original);
  return result;
}
void invalid(const std::string &s, FriPacketError error) {
  const auto r = inspect(s);
  CHECK(!r.valid());
  CHECK(r.error == error);
  CHECK(!r.measured_position.present && r.measured_position.count == 0);
  CHECK(!r.measured_torque.present && !r.commanded_torque.present && !r.external_torque.present);
  CHECK(!r.commanded_position.present && r.commanded_position.count == 0);
  CHECK(r.metadata_presence == 0);
}

void complete_and_absent() {
  auto r = inspect(full_packet());
  CHECK(r.valid());
  CHECK(r.measured_position.present && r.measured_position.count == 7);
  CHECK(r.measured_torque.present && r.measured_torque.count == 7);
  CHECK(r.commanded_torque.present && r.commanded_torque.count == 7);
  CHECK(r.external_torque.present && r.external_torque.count == 7);
  CHECK(r.commanded_position.present && r.commanded_position.count == 7);
  CHECK(r.metadata_presence == 255);
  // A second datagram must not inherit any availability from the first.
  r = inspect(header());
  CHECK(r.valid());
  CHECK(!r.measured_position.present && !r.measured_torque.present);
  CHECK(!r.commanded_torque.present && !r.external_torque.present);
  CHECK(!r.commanded_position.present);
  CHECK(r.metadata_presence == 0);
  for (unsigned mask = 0; mask < 16; ++mask) {
    std::string monitor = message(1, joints());
    if (mask & 1) monitor += message(2, joints());
    if (mask & 2) monitor += message(4, joints());
    if (mask & 4) monitor += message(5, joints());
    if (mask & 8) monitor += message(3, joints());
    r = inspect(packet(monitor));
    CHECK(r.valid());
    CHECK(r.measured_torque.present == bool(mask & 1));
    CHECK(r.commanded_torque.present == bool(mask & 2));
    CHECK(r.external_torque.present == bool(mask & 4));
    CHECK(r.commanded_position.present == bool(mask & 8));
  }
}

void repeated_encoding_and_values() {
  const std::string one = std::string(1, '\x09') + doubles(1, -1.);
  std::string unpacked;
  for (int i = 0; i < 7; ++i) unpacked += one;
  for (const auto &array : {joints(), unpacked, message(1, doubles(3)) + message(1, doubles(4)),
                            one + message(1, doubles(6))}) {
    const auto r = inspect(packet(message(1, array) + message(2, array)));
    CHECK(r.valid());
    CHECK(r.measured_position.count == 7 && r.measured_torque.count == 7);
  }
  for (double v : {0., -0., std::numeric_limits<double>::quiet_NaN(),
                   std::numeric_limits<double>::infinity()}) {
    const auto r = inspect(packet(message(2, joints(7, v))));
    CHECK(r.valid()); // Nonfinite validity is checked on SDK values in the later adapter.
    CHECK(r.measured_torque.present && r.measured_torque.count == 7);
  }
  for (uint32_t tag : {1, 2, 3, 4, 5}) {
    for (std::size_t count : {0, 6, 8})
      invalid(packet(message(tag, joints(count))), FriPacketError::InvalidArrayLength);
    invalid(packet(message(tag, "")), FriPacketError::InvalidArrayLength);
  }
}

void metadata_presence() {
  const auto time = message(3, message(15, integer(1, 0) + integer(2, 0)));
  const auto connection = message(4, integer(1, 0) + integer(2, 0));
  const std::array<std::pair<std::string, uint32_t>, 9> cases{{
    {time, c::ControllerTime}, {connection, c::Session | c::Quality},
    {message(4, integer(1, 0) + integer(2, 0) + integer(3, 0)), c::Session | c::Quality | c::SamplePeriod},
    {message(2, integer(7, 0)), c::Control}, {message(5, integer(10, 0)), c::Command},
    {message(2, integer(2, 3)), c::Safety}, {message(5, fixed64(12, .5)), c::Tracking},
    // Tracking performance 0 and a nonfinite value are data; range checks come later.
    {message(5, fixed64(12, 0.)), c::Tracking},
    {message(5, fixed64(12, std::numeric_limits<double>::quiet_NaN())), c::Tracking}
  }};
  for (const auto &item : cases) {
    const auto r = inspect(header() + item.first);
    CHECK(r.valid());
    CHECK(r.metadata_presence == item.second);
  }
  invalid(header() + message(4, integer(1, 4)), FriPacketError::MissingRequiredField);
  invalid(packet(message(15, integer(1, 0))), FriPacketError::MissingRequiredField);
  invalid(message(3, message(1, joints())), FriPacketError::MissingRequiredField);
  invalid(message(1, integer(1, 0x245142)), FriPacketError::MissingRequiredField);
  invalid(message(1, integer(1, 42) + integer(2, 1) + integer(3, 0)), FriPacketError::WrongMessageType);
  invalid(header() + message(4, integer(1, 4) + integer(2, 3) + integer(3, uint64_t(1) << 32)), FriPacketError::Malformed);
  invalid(header() + message(2, integer(2, uint64_t(1) << 32)), FriPacketError::Malformed);
  invalid(header() + message(5, integer(12, 1)), FriPacketError::Malformed); // trackingPerformance is a double
  invalid(header() + message(2, fixed64(2, 0.)), FriPacketError::Malformed); // safetyState is a varint
}

void duplicate_fields() {
  invalid(header() + header(), FriPacketError::DuplicateField);
  for (uint32_t tag : {1, 2, 3, 4, 5})
    invalid(packet(message(tag, joints()) + message(tag, joints())), FriPacketError::DuplicateField);
  for (const auto &part : {message(2, integer(7, 0)), message(3, ""),
                           message(4, integer(1, 4) + integer(2, 3)), message(5, integer(10, 1))})
    invalid(header() + part + part, FriPacketError::DuplicateField);
  invalid(packet(message(15, integer(1, 0) + integer(1, 0) + integer(2, 0))), FriPacketError::DuplicateField);
  invalid(packet(message(15, integer(1, 0) + integer(2, 0)) + message(15, integer(1, 0) + integer(2, 0))), FriPacketError::DuplicateField);
  invalid(header() + message(4, integer(1, 4) + integer(1, 4) + integer(2, 3)), FriPacketError::DuplicateField);
  invalid(header() + message(2, integer(7, 0) + integer(7, 0)), FriPacketError::DuplicateField);
  invalid(header() + message(5, integer(10, 1) + integer(10, 1)), FriPacketError::DuplicateField);
  invalid(header() + message(2, integer(2, 0) + integer(2, 0)), FriPacketError::DuplicateField);
  invalid(header() + message(5, fixed64(12, 1.) + fixed64(12, 1.)), FriPacketError::DuplicateField);
}

void malformed_lengths_and_wire() {
  const auto p = packet(message(1, joints()));
  for (std::size_t n = header().size() + 1; n < p.size(); ++n)
    invalid(p.substr(0, n), FriPacketError::Malformed);
  invalid(header() + std::string("\x1a\x80", 2), FriPacketError::Malformed);
  invalid(packet(message(1, message(1, std::string(55, '\0')))), FriPacketError::Malformed);
  invalid(packet(message(1, std::string("\x0a\xff\x7f", 3))), FriPacketError::Malformed);
  invalid(header() + std::string("\x1a", 1) + varint(UINT64_MAX), FriPacketError::Malformed);
  invalid(header() + std::string("\x80", 1), FriPacketError::Malformed);
  invalid(header() + std::string(10, '\xff') + '\x02', FriPacketError::Malformed);
  invalid(header() + std::string(1, '\0'), FriPacketError::Malformed);
  invalid(header() + integer(3, 0), FriPacketError::Malformed); // monitorData must be a message
  invalid(packet(integer(2, 0)), FriPacketError::Malformed); // torque must be a message
  invalid(packet(message(2, integer(1, 7))), FriPacketError::Malformed); // doubles must be fixed64
  invalid(header() + std::string(1, '\x33'), FriPacketError::Malformed); // groups unsupported
  invalid(header() + varint(uint64_t(1) << 32) + '\0', FriPacketError::Malformed);
}

void unrelated_fields_and_size_limit() {
  auto p = full_packet();
  // Unknown fields of all supported wire types, including opaque nested bytes.
  p += integer(100, UINT64_MAX) + message(101, std::string("\0\xff", 2));
  p += varint((102 << 3) | 1) + doubles(1, 3.);
  p += varint((103 << 3) | 5) + std::string(4, '\0');
  CHECK(inspect(p).valid());
  CHECK(inspect(packet(message(1, joints() + integer(9, 42)))).measured_position.count == 7);
  const auto base = full_packet();
  for (std::size_t n = 0; n < 1500; ++n) {
    p = base + message(100, std::string(n, 'x'));
    if (p.size() == 1500) break;
  }
  CHECK(p.size() == 1500 && inspect(p).valid());
  invalid(p + 'x', FriPacketError::InvalidInput);
  CHECK(!inspect_fri_packet(nullptr, 0).valid());
  CHECK(!inspect_fri_packet(nullptr, 1).valid());
  CHECK(!inspect_fri_packet(p.data(), 0).valid());
  CHECK(!inspect_fri_packet(p.data(), std::numeric_limits<std::size_t>::max()).valid());
}

int main() {
  complete_and_absent();
  repeated_encoding_and_values();
  metadata_presence();
  duplicate_fields();
  malformed_lengths_and_wire();
  unrelated_fields_and_size_limit();
  if (failures) { std::cerr << failures << " failed checks\n"; return 1; }
  std::cout << "Seven zero measured torques: present, count=7\n"
               "Next packet without measured torque: unavailable, count=0\n"
               "FRI packet availability checks passed (synthetic packets, no hardware).\n";
}
