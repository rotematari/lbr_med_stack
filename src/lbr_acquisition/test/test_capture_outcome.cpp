#include "lbr_acquisition/capture_outcome.hpp"
#include "lbr_acquisition/offline_batch_writer.hpp"
#include <cassert>
#include <cerrno>
#include <fstream>
#include <limits>
#include <string>
#include <unistd.h>

using namespace lbr_acquisition;
namespace c = lbr_acquisition::capture;
namespace fs = std::filesystem;

static c::Event state(uint64_t id, uint64_t run = 9007199254740993ULL) {
  c::Event e{};
  e.kind = c::Kind::State;
  e.run_id = run;
  e.id = e.state_id = id;
  e.host_monotonic_ns = 9007199254740992ULL + id;
  e.position = {.125, -.25, .375, -.5, .625, -.75, .875};
  return e;
}
static std::string read(const fs::path &p) {
  std::ifstream f(p, std::ios::binary);
  assert(f);
  return {std::istreambuf_iterator<char>(f), {}};
}
template <class F> static void invalid(F f) {
  bool caught = false;
  try { f(); } catch (const InvalidInput &) { caught = true; }
  assert(caught);
}
static c::Status drain(c::Recorder &r, OfflineBatchWriter &w) {
  for (int i = 0; i < 10 && !r.status().writer_done; ++i) r.writer_step(w);
  assert(r.status().writer_done);
  return r.status();
}
struct OutcomeIO : FileOps {
  enum Fault { None, Short, EintrOnce, Zero, Disk, Close } fault = None;
  bool once = false;
  bool outcome(int fd) {
    char p[1024]{};
    const auto n = readlink(("/proc/self/fd/" + std::to_string(fd)).c_str(), p, 1023);
    return n > 0 && std::string(p).find(".capture.json.partial") != std::string::npos;
  }
  ssize_t write(int fd, const void *p, size_t n) override {
    if (outcome(fd)) {
      if (fault == EintrOnce && !once) { once = true; errno = EINTR; return -1; }
      if (fault == Zero) return 0;
      if (fault == Disk) { errno = ENOSPC; return -1; }
      if (fault == Short) n = std::min(n, size_t(3));
    }
    return FileOps::write(fd, p, n);
  }
  int close(int fd) override {
    const bool hit = outcome(fd);
    const int result = FileOps::close(fd);
    if (hit && fault == Close) { errno = EIO; return -1; }
    return result;
  }
};
struct QualityCloseIO : FileOps {
  int close(int fd) override {
    char p[1024]{};
    const auto n = readlink(("/proc/self/fd/" + std::to_string(fd)).c_str(), p, 1023);
    const bool hit = n > 0 && std::string(p).find("quality.json.partial") != std::string::npos;
    const int result = FileOps::close(fd);
    if (hit) { errno = EIO; return -1; }
    return result;
  }
};
static fs::path sidecar(const fs::path &p) { return p.string() + ".capture.json"; }

int main(int argc, char **argv) {
  assert(argc == 2 || (argc == 4 && std::string(argv[1]) == "--fixtures"));
  const fs::path descriptor(argc == 2 ? argv[1] : argv[3]);
  char pattern[] = "/tmp/acq-outcome-XXXXXX";
  const fs::path root = argc == 4 ? fs::path(argv[2]) : fs::path(mkdtemp(pattern));
  assert(!root.empty());
  fs::create_directories(root);
  FileOps io;
  const uint64_t run = 9007199254740993ULL;
  {
    const auto out = root / "complete";
    OfflineBatchWriter w(run, out, descriptor, io);
    c::Recorder r({run, 1000, 1000, 10, 4});
    assert(r.start(run, 9007199254740992ULL));
    assert(r.offer(state(1)) == c::Offer::Accepted);
    assert(r.stop(run));
    auto st = drain(r, w);
    write_capture_outcome(w, st, io);
    const auto bytes = read(sidecar(out));
    assert(bytes.find("\"capture_run_id\":\"9007199254740993\"") != std::string::npos);
    assert(bytes.find("\"phase\":\"Complete\"") != std::string::npos);
    assert(bytes.find("\"written\":\"1\"") != std::string::npos);
    assert(bytes.find("\"manifest.json\":{") != std::string::npos);
    invalid([&] { write_capture_outcome(w, st, io); });
    assert(read(sidecar(out)) == bytes);
    auto bad = st; bad.writer_done = false;
    invalid([&] { write_capture_outcome(w, bad, io); });
    bad = st; bad.enqueued = 2;
    invalid([&] { write_capture_outcome(w, bad, io); });
  }
  {
    const auto out = root / "overflow";
    OfflineBatchWriter w(run, out, descriptor, io);
    c::Recorder r({run, 1000, 1000, 10, 1});
    assert(r.start(run, 9007199254740992ULL));
    assert(r.offer(state(1)) == c::Offer::Accepted);
    assert(r.offer(state(2)) == c::Offer::Rejected);
    auto st = drain(r, w);
    assert(st.phase == c::Phase::Failed && w.state() == OfflineBatchWriter::State::Finalized);
    write_capture_outcome(w, st, io);
    assert(read(sidecar(out)).find("\"reason\":\"Overflow\"") != std::string::npos);
  }
  {
    const auto out = root / "duration";
    OfflineBatchWriter w(run, out, descriptor, io);
    c::Recorder r({run, 10, 1000, 10, 4});
    assert(r.start(run, 9007199254740992ULL));
    assert(r.offer(state(1)) == c::Offer::Accepted);
    r.tick(9007199254741003ULL);
    auto st = drain(r, w);
    assert(st.phase == c::Phase::Complete && st.reason == c::Reason::Duration);
    write_capture_outcome(w, st, io);
  }
  {
    const auto out = root / "empty-complete";
    OfflineBatchWriter w(run, out, descriptor, io);
    c::Recorder r({run, 1000, 1000, 10, 4});
    assert(r.stop(run));
    auto st = drain(r, w);
    assert(st.phase == c::Phase::Complete && !st.has_first);
    write_capture_outcome(w, st, io);
  }
  {
    const auto out = root / "nonfinite";
    OfflineBatchWriter w(run, out, descriptor, io);
    c::Recorder r({run, 1000, 1000, 10, 4});
    assert(r.start(run, 9007199254740992ULL));
    auto e = state(1); e.position[0] = std::numeric_limits<double>::infinity();
    e.nonfinite_joint_mask = 1;
    assert(r.offer(e) == c::Offer::Accepted);
    assert(r.stop(run));
    write_capture_outcome(w, drain(r, w), io);
  }
  {
    const auto out = root / "quality-close";
    QualityCloseIO fault;
    OfflineBatchWriter w(run, out, descriptor, fault);
    c::Recorder r({run, 1000, 1000, 10, 4});
    assert(r.start(run, 9007199254740992ULL));
    assert(r.offer(state(1)) == c::Offer::Accepted);
    assert(r.stop(run));
    auto st = drain(r, w);
    assert(st.phase == c::Phase::Failed && w.state() == OfflineBatchWriter::State::Failed);
    assert(fs::exists(out / "signals.mcap"));
    auto bad = st;
    bad.failure_flags = 1u << static_cast<unsigned>(c::Reason::Overflow);
    invalid([&] { write_capture_outcome(w, bad, io); });
    auto shifted = st;
    ++shifted.first_state_ns;
    ++shifted.last_state_ns;
    invalid([&] { write_capture_outcome(w, shifted, io); });
    assert(!fs::exists(sidecar(out)));
    write_capture_outcome(w, st, io);
  }
  {
    const auto out = root / "no-data";
    OfflineBatchWriter w(run, out, descriptor, io);
    c::Recorder r({run, 1000, 1000, 10, 4});
    assert(r.start(run, 1)); r.tick(1001);
    write_capture_outcome(w, drain(r, w), io);
    assert(read(sidecar(out)).find("\"reason\":\"NoData\"") != std::string::npos);
  }
  {
    const auto out = root / "pre-directory-failure";
    OfflineBatchWriter w(run, out, root / "missing-descriptor", io);
    c::Recorder r({run, 1000, 1000, 10, 4});
    assert(r.start(run, 9007199254740992ULL));
    assert(r.offer(state(1)) == c::Offer::Accepted);
    assert(r.stop(run));
    auto st = drain(r, w);
    assert(st.phase == c::Phase::Failed && !fs::exists(out));
    fs::create_directory_symlink(root, out);
    invalid([&] { write_capture_outcome(w, st, io); });
    fs::remove(out);
    write_capture_outcome(w, st, io);
    assert(fs::exists(sidecar(out)));
  }
  for (auto fault : {OutcomeIO::Short, OutcomeIO::EintrOnce, OutcomeIO::Zero,
                     OutcomeIO::Disk, OutcomeIO::Close}) {
    const auto out = root / ("sidecar-fault-" + std::to_string(fault));
    OfflineBatchWriter w(run, out, descriptor, io);
    c::Recorder r({run, 1000, 1000, 10, 4});
    assert(r.start(run, 9007199254740992ULL));
    assert(r.offer(state(1)) == c::Offer::Accepted);
    assert(r.stop(run));
    auto st = drain(r, w);
    OutcomeIO injected; injected.fault = fault;
    if (fault == OutcomeIO::Short || fault == OutcomeIO::EintrOnce) {
      write_capture_outcome(w, st, injected);
      assert(fs::exists(sidecar(out)));
    } else {
      bool caught = false;
      try { write_capture_outcome(w, st, injected); } catch (const IoError &) { caught = true; }
      assert(caught && !fs::exists(sidecar(out)));
      assert(fs::exists(sidecar(out).string() + ".partial"));
      assert(r.status().phase == c::Phase::Complete);
    }
  }
  if (argc == 2) fs::remove_all(root);
}
