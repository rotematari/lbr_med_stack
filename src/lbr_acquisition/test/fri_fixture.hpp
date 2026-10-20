// PHD-FRI-RECORDER: shared in-memory FRI 1.15 fixture for driver/recording tests.
// One translation unit per test executable includes this (inline globals).
#pragma once
#include "lbr_acquisition/fri_adapter.hpp"
#include "lbr_acquisition/streaming_capture_session.hpp"
#include "lbr_fri_ros2/async_client.hpp"
#include "friClientApplication.h"
#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace fri_fixture {
using namespace lbr_acquisition;
using Joints = std::array<double, 7>;
inline int failures = 0;
} // namespace fri_fixture
#define CHECK(x) do { if (!(x)) { std::cerr << __LINE__ << ": " #x "\n"; ++fri_fixture::failures; } } while (false)
namespace fri_fixture {
// Literal wire tags checked against the unmodified SDK 1.15 FRIMessages.pb.h/.c.
inline std::string varint(uint64_t v) {
  std::string s;
  do { const auto b = v & 127; v >>= 7; s += char(b | (v ? 128 : 0)); } while (v);
  return s;
}
inline std::string integer(unsigned tag, uint64_t v) { return varint(tag << 3) + varint(v); }
inline std::string message(unsigned tag, const std::string &s) {
  return varint((tag << 3) | 2) + varint(s.size()) + s;
}
inline Joints values(double base) {
  Joints v{};
  for (size_t j = 0; j < v.size(); ++j) v[j] = base + double(j) * .01;
  return v;
}
inline std::string joints(const Joints &v) {
  std::string bytes;
  for (double d : v) {
    uint64_t bits; std::memcpy(&bits, &d, 8);
    for (unsigned b = 0; b < 8; ++b) bytes += char(bits >> (8 * b));
  }
  return message(1, bytes);
}
inline std::string packet(unsigned seq, unsigned session, unsigned torques = 7) {
  const auto q = values(.1 + .001 * seq);
  std::string monitor = message(1, joints(q)) + message(3, joints(values(.2)));
  if (torques & 1) monitor += message(2, joints(values(10. + seq)));
  if (torques & 2) monitor += message(4, joints(values(-3. - seq)));
  if (torques & 4) monitor += message(5, joints(values(.5 + seq)));
  monitor += message(15, integer(1, 42 + seq / 1000) + integer(2, (seq % 1000) * 1000000));
  std::string drives;
  for (int j = 0; j < 7; ++j) drives += integer(5, 2);
  return message(1, integer(1, 0x245142) + integer(2, seq) + integer(3, 0)) +
         message(2, integer(1, 7) + integer(2, 0) + drives + integer(6, 0) + integer(7, 0)) +
         message(3, monitor) +
         message(4, integer(1, session) + integer(2, 3) + integer(3, 1) + integer(4, 2)) +
         message(5, message(1, joints(q)) + integer(10, 1) + integer(11, 1));
}

struct MemoryConnection : KUKA::FRI::IConnection {
  std::vector<std::string> inputs, sent;
  size_t next = 0;
  bool opened = true;
  bool open(int, const char *) override { opened = true; return true; }
  void close() override { opened = false; }
  bool isOpen() const override { return opened; }
  int receive(char *out, int capacity) override {
    if (next == inputs.size()) return 0;
    const auto &s = inputs[next++];
    if (s.size() > size_t(capacity)) return -1;
    std::memcpy(out, s.data(), s.size());
    return int(s.size());
  }
  bool send(const char *p, int n) override { sent.emplace_back(p, n); return true; }
};
inline uint64_t now = 1000000;
inline uint64_t clock_ns() noexcept { return now += 100; }
// Real SDK client and driver over an in-memory connection observed by `observer`.
// Members are destroyed in reverse order: application, client, then connection.
struct DriverRig {
  std::unique_ptr<KUKA::FRI::IConnection> connection;
  MemoryConnection *wire = nullptr;
  std::unique_ptr<lbr_fri_ros2::AsyncClient> client;
  std::unique_ptr<KUKA::FRI::ClientApplication> app;
  DriverRig(const std::shared_ptr<RecorderAdapter> &observer, unsigned cycles, bool hooks = true) {
    auto memory = std::make_unique<MemoryConnection>();
    wire = memory.get();
    memory->inputs.push_back(packet(0, 2));
    for (unsigned i = 1; i < cycles; ++i) memory->inputs.push_back(packet(i, 4));
    connection = std::make_unique<ObservedConnection>(std::move(memory), observer);
    lbr_fri_ros2::CommandGuardParameters guard;
    guard.min_positions.fill(-2.); guard.max_positions.fill(2.);
    guard.max_velocities.fill(10000.); guard.max_torques.fill(10000.);
    client = std::make_unique<lbr_fri_ros2::AsyncClient>(KUKA::FRI::POSITION, .01, guard,
        "default", lbr_fri_ros2::StateGuardParameters{},
        lbr_fri_ros2::StateInterfaceParameters{.04, .04}, true);
    if (hooks) client->set_recorder(observer);
    app = std::make_unique<KUKA::FRI::ClientApplication>(*connection, *client);
  }
  void step() { now = StreamingCaptureSession::now_ns(); CHECK(app->step()); }
};
} // namespace fri_fixture
