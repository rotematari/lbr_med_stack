#include "lbr_acquisition/offline_writer.hpp"
#include <cassert>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <limits>
#include <unistd.h>
using namespace lbr_acquisition;
struct FaultIO : FileOps {
  enum Kind {
    Short,
    Interrupt,
    Zero,
    Full,
    Close,
    Log,
    QualityClose,
    FullClose
  } kind;
  int calls = 0, closes = 0, writes_after_failure = 0;
  bool failed = false;
  explicit FaultIO(Kind k) : kind(k) {}
  bool target(int fd) {
    char path[1024]{};
    auto n = ::readlink(("/proc/self/fd/" + std::to_string(fd)).c_str(), path,
                        sizeof(path) - 1);
    return n > 0 && std::string(path).find(kind == Log ? "terminal.log"
                                           : kind == QualityClose
                                               ? "quality.json"
                                               : "signals.mcap.partial") !=
                        std::string::npos;
  }
  ssize_t write(int fd, const void *p, size_t n) override {
    if (!target(fd))
      return FileOps::write(fd, p, n);
    if (failed)
      ++writes_after_failure;
    ++calls;
    if (kind == Interrupt && calls == 1) {
      errno = EINTR;
      return -1;
    }
    if (kind == Zero || kind == Log) {
      failed = true;
      return 0;
    }
    if ((kind == Full || kind == FullClose) && calls >= 4) {
      failed = true;
      errno = ENOSPC;
      return -1;
    }
    return FileOps::write(fd, p, kind == Short ? std::min(n, size_t(3)) : n);
  }
  int close(int fd) override {
    bool hit = target(fd);
    if (hit)
      ++closes;
    int ret = FileOps::close(fd);
    if (hit && (kind == Close || kind == QualityClose || kind == FullClose)) {
      failed = true;
      errno = EIO;
      return -1;
    }
    return ret;
  }
};
int main(int argc, char **argv) {
  assert(argc == 2);
  auto root = std::filesystem::temp_directory_path() /
              ("acq-storage-" + std::to_string(getpid()));
  assert(std::filesystem::create_directory(root));
  const auto events = synthetic_events();
  for (auto k : {FaultIO::Short, FaultIO::Interrupt, FaultIO::Zero,
                 FaultIO::Full, FaultIO::Close, FaultIO::Log}) {
    FaultIO io(k);
    auto out = root / std::to_string(k);
    bool threw = false;
    try {
      write_fixture(out, events, argv[1], io);
    } catch (const IoError &) {
      threw = true;
    }
    bool success = k == FaultIO::Short || k == FaultIO::Interrupt;
    assert(threw != success);
    assert(io.writes_after_failure == 0 && io.closes == 1);
    assert(std::filesystem::exists(out / "signals.mcap") == success);
    if (success)
      assert(std::filesystem::exists(out / "quality.json"));
    else if (k != FaultIO::Log)
      assert(std::filesystem::exists(out / "signals.mcap.partial"));
  }
  {
    FaultIO io(FaultIO::QualityClose);
    auto out = root / "quality-close";
    bool threw = false;
    try {
      write_fixture(out, events, argv[1], io);
    } catch (const IoError &) {
      threw = true;
    }
    assert(threw);
    if (std::filesystem::exists(out / "quality.json")) {
      std::ifstream f(out / "quality.json");
      std::string quality((std::istreambuf_iterator<char>(f)), {});
      assert(quality.find("\"finalization\":\"complete\"") ==
             std::string::npos);
    }
  }
  {
    FaultIO io(FaultIO::FullClose);
    auto out = root / "write-and-close";
    try {
      write_fixture(out, events, argv[1], io);
      assert(false);
    } catch (const IoError &error) {
      assert(std::string(error.what()).find("write") != std::string::npos);
    }
    assert(io.closes == 1 && io.writes_after_failure == 0);
    std::ifstream f(out / "quality.json");
    std::string report((std::istreambuf_iterator<char>(f)), {});
    assert(report.find("cleanup close errno") != std::string::npos);
  }
  {
    // The profile permits 10,000 full State records, including quality
    // anomalies. Only the manifest has the 64 KiB limit; quality must retain
    // these issues.
    std::vector<OfflineEvent> maximum;
    for (uint64_t i = 0; i < 10000; ++i) {
      lbr::acquisition::v1::State value;
      value.set_schema_version(1);
      value.set_state_id(UINT64_MAX - 9999 + i);
      value.set_host_monotonic_ns(UINT64_MAX);
      value.set_controller_sec(UINT32_MAX);
      value.set_controller_nsec(UINT32_MAX);
      value.set_session_state(UINT32_MAX);
      value.set_connection_quality(UINT32_MAX);
      value.set_control_mode(UINT32_MAX);
      value.set_client_command_mode(UINT32_MAX);
      value.set_reported_sample_period_s(std::numeric_limits<double>::max());
      value.set_nonfinite_joint_mask(0);
      for (int j = 0; j < 7; ++j)
        value.add_q_measured_rad(std::numeric_limits<double>::max());
      maximum.emplace_back(value);
    }
    write_fixture(root / "maximum", maximum, argv[1]);
    assert(std::filesystem::file_size(root / "maximum" / "signals.mcap") <
           16777216);
    assert(std::filesystem::file_size(root / "maximum" / "quality.json") >
           65536);
    // Missing parent / inaccessible output fails without creating a successful
    // run.
    bool inaccessible = false;
    try {
      write_fixture(root / "missing-parent" / "run", events, argv[1]);
    } catch (const IoError &) {
      inaccessible = true;
    }
    assert(inaccessible);
  }
  if (::geteuid() != 0) {
    auto locked = root / "read-only";
    std::filesystem::create_directory(locked);
    std::filesystem::permissions(locked,
                                 std::filesystem::perms::owner_read |
                                     std::filesystem::perms::owner_exec);
    bool denied = false;
    try {
      write_fixture(locked / "run", events, argv[1]);
    } catch (const IoError &) {
      denied = true;
    }
    std::filesystem::permissions(locked, std::filesystem::perms::owner_all);
    assert(denied && !std::filesystem::exists(locked / "run"));
  }
  auto old = root / "existing";
  std::filesystem::create_directory(old);
  {
    std::ofstream f(old / "sentinel");
    f << "preserve";
  }
  bool rejected = false;
  try {
    write_fixture(old, events, argv[1]);
  } catch (const ExistingOutput &) {
    rejected = true;
  }
  assert(rejected && std::filesystem::file_size(old / "sentinel") == 8);
  std::filesystem::remove_all(root);
}
