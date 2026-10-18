#include "lbr_acquisition/capture_encoding.hpp"
#include "lbr_acquisition/offline_batch_writer.hpp"
#include "lbr_acquisition/readback.hpp"
#include <algorithm>
#include <cassert>
#include <cerrno>
#include <fstream>
#include <type_traits>
#include <unistd.h>
using namespace lbr_acquisition;
namespace c = lbr_acquisition::capture;
using S = OfflineBatchWriter::State;
std::string read(const std::filesystem::path &p) {
  std::ifstream f(p, std::ios::binary);
  assert(f);
  return {std::istreambuf_iterator<char>(f), {}};
}
c::Event event(uint64_t id = 1) {
  c::Event e{};
  e.kind = c::Kind::State;
  e.run_id = 17;
  e.id = e.state_id = id;
  e.host_monotonic_ns = 9007199254740992ULL + id;
  e.position = {.125, -0., -.25, .375, -.5, .625, -.75};
  return e;
}
std::vector<c::Event> events() {
  auto a = event(), b = event(2), s = event(3);
  b.position[0] = .25;
  s.kind = c::Kind::Submission;
  s.state_id = 2;
  s.position[0] = .5;
  return {a, b, s};
}
template <class E = InvalidInput, class F> void throws(F f) {
  bool threw = false;
  try {
    f();
  } catch (const E &) {
    threw = true;
  }
  assert(threw);
}
void drain(c::Recorder &r, c::Writer &w) {
  for (int i = 0; i < 20 && !r.status().writer_done; ++i)
    r.writer_step(w);
  assert(r.status().writer_done);
}
void feed(c::Recorder &r) {
  assert(r.start(17, 9007199254740992ULL));
  for (auto e : events())
    assert(r.offer(e) == c::Offer::Accepted);
  assert(r.stop(17));
}
struct FaultIO : FileOps {
  enum Kind { Short, Disk, Close, QualityClose } kind;
  explicit FaultIO(Kind k) : kind(k) {}
  bool target(int fd) {
    char p[1024]{};
    auto n = readlink(("/proc/self/fd/" + std::to_string(fd)).c_str(), p, 1023);
    return n > 0 &&
           std::string(p).find(kind == QualityClose ? "quality.json.partial"
                                                    : "signals.mcap.partial") !=
               std::string::npos;
  }
  ssize_t write(int fd, const void *p, size_t n) override {
    if (target(fd) && kind == Disk) {
      errno = ENOSPC;
      return -1;
    }
    return FileOps::write(fd, p, kind == Short ? std::min(n, size_t(3)) : n);
  }
  int close(int fd) override {
    bool hit = target(fd);
    int result = FileOps::close(fd);
    if (hit && (kind == Close || kind == QualityClose)) {
      errno = EIO;
      return -1;
    }
    return result;
  }
};
int main(int argc, char **argv) {
  assert(argc == 2);
  static_assert(!std::is_copy_constructible_v<OfflineBatchWriter>);
  static_assert(!std::is_move_constructible_v<OfflineBatchWriter>);
  char pattern[] = "/tmp/acq-batch-XXXXXX";
  const auto temp = mkdtemp(pattern);
  assert(temp);
  const std::filesystem::path root(temp), descriptor(argv[1]);
  FileOps io;
  throws([&] { OfflineBatchWriter w(0, root / "zero", descriptor, io); });
  assert(!std::filesystem::exists(root / "zero"));
  {
    auto out = root / "success";
    OfflineBatchWriter w(17, out, descriptor, io);
    c::Recorder r({17, 1000, 1000, 10, 4});
    feed(r);
    drain(r, w);
    assert(r.status().phase == c::Phase::Complete && r.status().written == 3);
    assert(w.state() == S::Finalized && w.staged_events().size() == 3);
    verify_container(out / "signals.mcap", encode_capture_batch(events(), 17),
                     read(descriptor));
    const auto original = read(out / "signals.mcap");
    assert(!r.writer_step(w));
    throws([&] { w.finish(); });
    throws([&] { w.write(event()); });
    assert(w.state() == S::Finalized && read(out / "signals.mcap") == original);
  }
  {
    OfflineBatchWriter w(17, root / "empty", descriptor, io);
    w.finish();
    verify_container(root / "empty/signals.mcap", {}, read(descriptor));
  }
  {
    auto out = root / "destructor";
    {
      OfflineBatchWriter w(17, out, descriptor, io);
      auto e = event();
      w.write(e);
      e.position.fill(99);
      assert(w.staged_events()[0].position[0] == .125);
    }
    assert(!std::filesystem::exists(out));
  }
  for (int mode = 0; mode < 2; ++mode) {
    auto out = root / ("rejected" + std::to_string(mode));
    OfflineBatchWriter w(17, out, descriptor, io);
    w.write(event());
    size_t expected = 1;
    if (mode == 0) {
      auto wrong = event(2);
      wrong.run_id = 18;
      throws([&] { w.write(wrong); });
    } else {
      for (uint64_t id = 2; id <= 10000; ++id)
        w.write(event(id));
      expected = 10000;
      throws([&] { w.write(event(10001)); });
    }
    assert(w.state() == S::Failed && w.staged_events().size() == expected);
    throws([&] { w.finish(); });
    throws([&] { w.write(event()); });
    assert(!std::filesystem::exists(out));
  }
  {
    OfflineBatchWriter w(17, root / "encoding", descriptor, io);
    auto bad = event();
    bad.nonfinite_joint_mask = 1;
    w.write(bad);
    throws([&] { w.finish(); });
    assert(w.state() == S::Failed && w.staged_events().size() == 1);
    assert(!std::filesystem::exists(root / "encoding"));
  }
  {
    // Writer rejects run identity; core considers the records otherwise valid.
    OfflineBatchWriter w(18, root / "write-failure", descriptor, io);
    c::Recorder r({17, 1000, 1000, 10, 4});
    feed(r);
    drain(r, w);
    const auto st = r.status();
    assert(st.phase == c::Phase::Failed && st.reason == c::Reason::Manual);
    assert(st.write_uncertain == 1 && st.abandoned == 2 && st.written == 0);
    assert(w.state() == S::Failed && w.staged_events().empty());
    assert(!std::filesystem::exists(root / "write-failure"));
  }
  {
    auto out = root / "overflow";
    OfflineBatchWriter w(17, out, descriptor, io);
    c::Recorder r({17, 1000, 1000, 10, 1});
    assert(r.start(17, 9007199254740992ULL));
    assert(r.offer(event()) == c::Offer::Accepted);
    assert(r.offer(event(2)) == c::Offer::Rejected);
    drain(r, w);
    auto st = r.status();
    assert(st.phase == c::Phase::Failed && st.reason == c::Reason::Overflow);
    assert(st.written == 1 && st.rejected == 1 && w.state() == S::Finalized);
    assert(st.written == w.staged_events().size());
    verify_container(out / "signals.mcap", encode_capture_batch({event()}, 17),
                     read(descriptor));
    auto manifest = read(out / "manifest.json"),
         quality = read(out / "quality.json");
    assert(manifest.find("\"profile\":\"offline-fixture-v1\"") !=
           std::string::npos);
    assert(manifest.find("\"source\":\"synthetic\"") != std::string::npos);
    assert(manifest.find("\"run_id\":\"17\"") == std::string::npos);
    assert(quality.find("\"container_complete\":true") != std::string::npos);
    assert(quality.find("\"finalization\":\"complete\"") != std::string::npos);
  }
  for (auto kind :
       {FaultIO::Short, FaultIO::Disk, FaultIO::Close, FaultIO::QualityClose}) {
    auto out = root / ("io" + std::to_string(kind));
    FaultIO fault(kind);
    OfflineBatchWriter w(17, out, descriptor, fault);
    c::Recorder r({17, 1000, 1000, 10, 4});
    feed(r);
    drain(r, w);
    auto st = r.status();
    assert(st.reason == c::Reason::Manual && st.written == 3);
    if (kind == FaultIO::Short) {
      assert(st.phase == c::Phase::Complete && w.state() == S::Finalized);
    } else {
      assert(st.phase == c::Phase::Failed && w.state() == S::Failed);
      assert(st.failure_flags &
             (1u << static_cast<unsigned>(c::Reason::WriterFailure)));
      const auto q = read(out / "quality.json");
      assert(q.find("\"finalization\":\"failed\"") != std::string::npos);
      assert(std::filesystem::exists(out / (kind == FaultIO::QualityClose
                                                ? "signals.mcap"
                                                : "signals.mcap.partial")));
      throws([&] { w.finish(); });
      assert(read(out / "quality.json") == q);
    }
  }
  {
    auto out = root / "existing";
    std::filesystem::create_directory(out);
    {
      std::ofstream f(out / "sentinel");
      f << "preserve";
    }
    OfflineBatchWriter w(17, out, descriptor, io);
    w.write(event());
    throws<ExistingOutput>([&] { w.finish(); });
    assert(read(out / "sentinel") == "preserve" &&
           std::distance(std::filesystem::directory_iterator(out),
                         std::filesystem::directory_iterator{}) == 1);
  }
  std::filesystem::remove_all(root);
}
