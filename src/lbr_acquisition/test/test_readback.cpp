#include "lbr_acquisition/readback.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <unistd.h>

using namespace lbr_acquisition;
namespace fs = std::filesystem;
static std::string bytes(const fs::path &p) {
  std::ifstream f(p, std::ios::binary);
  return {std::istreambuf_iterator<char>(f), {}};
}
static void save(const fs::path &p, const std::string &b) {
  std::ofstream f(p, std::ios::binary);
  f.write(b.data(), b.size());
}
static void reject(const fs::path &p, const std::vector<OfflineEvent> &events,
                   const std::string &descriptor,
                   const std::string &reason = "") {
  try {
    verify_container(p, events, descriptor);
  } catch (const IoError &error) {
    if (!reason.empty() &&
        std::string(error.what()).find(reason) == std::string::npos)
      throw std::runtime_error("wrong rejection: " + std::string(error.what()));
    return;
  }
  throw std::runtime_error("corrupt container accepted");
}
int main(int argc, char **argv) {
  if (argc != 2)
    return 2;
  const auto root = fs::temp_directory_path() /
                    ("lbr-readback-test-" + std::to_string(getpid()));
  fs::create_directory(root);
  try {
    lbr::acquisition::v1::State state;
    state.set_schema_version(1);
    state.set_state_id(1);
    state.set_host_monotonic_ns(9007199254740993ULL);
    state.set_nonfinite_joint_mask(0);
    for (int j = 0; j < 7; ++j)
      state.add_q_measured_rad(j * 0.125);
    const std::vector<OfflineEvent> events{state};
    write_fixture(root / "run", events, argv[1]);
    const auto descriptor = bytes(argv[1]);
    const auto good = bytes(root / "run/signals.mcap");
    verify_container(root / "run/signals.mcap", events, descriptor);
    write_fixture(root / "empty", {}, argv[1]);
    verify_container(root / "empty/signals.mcap", {}, descriptor);
    const auto bad = root / "bad.mcap";
    // Locate outer records without reusing the production parser.
    std::size_t chunk_offset = 0, end_offset = 0;
    for (std::size_t offset = 8; offset + 9 <= good.size();) {
      std::uint64_t length = 0;
      for (int j = 0; j < 8; ++j)
        length |=
            std::uint64_t(static_cast<unsigned char>(good[offset + 1 + j]))
            << (8 * j);
      if (good[offset] == 6)
        chunk_offset = offset;
      if (good[offset] == 15) {
        end_offset = offset;
        break;
      }
      offset += 9 + length;
    }
    if (!chunk_offset || !end_offset)
      throw std::runtime_error("fixture missing records");
    auto corrupt_crc = good;
    corrupt_crc[end_offset + 9] ^= 1;
    save(bad, corrupt_crc);
    reject(bad, events, descriptor, "data checksum mismatch");
    corrupt_crc = good;
    for (int j = 0; j < 4; ++j)
      corrupt_crc[chunk_offset + 9 + 24 + j] = 0;
    save(bad, corrupt_crc);
    reject(bad, events, descriptor, "chunk checksum mismatch");
    corrupt_crc = good;
    for (int j = 0; j < 4; ++j)
      corrupt_crc[end_offset + 9 + j] = 0;
    save(bad, corrupt_crc);
    reject(bad, events, descriptor, "data checksum mismatch");
    save(bad, good + "garbage");
    reject(bad, events, descriptor);
    save(bad, good.substr(0, good.size() - 1));
    reject(bad, events, descriptor);
    auto changed = good;
    changed[0] ^= 1;
    save(bad, changed);
    reject(bad, events, descriptor);
    changed = good;
    changed[9] = static_cast<char>(0xff);
    changed[10] = static_cast<char>(0xff);
    changed[11] = static_cast<char>(0xff);
    save(bad, changed);
    reject(bad, events, descriptor);
    save(bad, good);
    reject(bad, events, descriptor + "x");
    auto wrong = events;
    std::get<lbr::acquisition::v1::State>(wrong[0]).set_host_monotonic_ns(42);
    reject(bad, wrong, descriptor);
    // A byte inside the data section cannot be silently accepted.
    changed = good;
    changed[good.size() / 2] ^= 1;
    save(bad, changed);
    reject(bad, events, descriptor);
    fs::remove_all(root);
    std::cout << "native readback valid and corruption cases passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    fs::remove_all(root);
    return 1;
  }
}
