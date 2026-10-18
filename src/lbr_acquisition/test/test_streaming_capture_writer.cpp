#include "lbr_acquisition/streaming_capture_writer.hpp"
#include "lbr_acquisition/capture_encoding.hpp"
#include "lbr_acquisition/readback.hpp"
#include "lbr_acquisition/capture_outcome.hpp"
#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unistd.h>
using namespace lbr_acquisition;
namespace fs = std::filesystem;
int main(int argc, char **argv) {
  if (argc == 4 && std::string(argv[1]) == "--hold-prefix") {
    FileOps io;
    capture::Event e{};
    e.run_id = 17; e.kind = capture::Kind::State; e.id = e.state_id = 1;
    e.host_monotonic_ns = 9007199254740993ULL;
    e.position = {.125, -.25, .375, -.5, .625, -.75, .875};
    StreamingCaptureWriter writer(17, argv[2], argv[3], io);
    writer.write(e);
    std::cout << "READY" << std::endl;
    std::string line;
    std::getline(std::cin, line);
    writer.finish();
    return 0;
  }
  assert(argc == 2);
  char pattern[] = "/tmp/acq-stream-writer-XXXXXX";
  fs::path root(mkdtemp(pattern));
  assert(!root.empty());
  FileOps io;
  bool invalid = false;
  try { StreamingCaptureWriter bad(0, root / "invalid", argv[1], io); }
  catch (const InvalidInput &) { invalid = true; }
  assert(invalid && !fs::exists(root / "invalid"));
  capture::Event e{};
  e.run_id = 17; e.kind = capture::Kind::State; e.id = e.state_id = 1;
  e.host_monotonic_ns = 9007199254740993ULL;
  e.position = {.125, -.25, .375, -.5, .625, -.75, .875};
  const auto run = root / "run";
  StreamingCaptureWriter writer(17, run, argv[1], io);
  assert(fs::exists(run / "signals.mcap.partial"));
  auto before = fs::file_size(run / "signals.mcap.partial");
  writer.write(e);
  assert(fs::file_size(run / "signals.mcap.partial") > before);
  assert(writer.staged_events().size() == 1);
  writer.finish();
  assert(writer.state() == StreamingCaptureWriter::State::Finalized);
  assert(!fs::exists(run / "signals.mcap.partial"));
  std::ifstream f(argv[1], std::ios::binary);
  std::string descriptor(std::istreambuf_iterator<char>{f}, {});
  verify_container(run / "signals.mcap", encode_capture_batch({e}, 17), descriptor);
  capture::Recorder recorder({17, 1000, 1000, 10, 4});
  assert(recorder.start(17, e.host_monotonic_ns));
  assert(recorder.offer(e) == capture::Offer::Accepted);
  assert(recorder.stop(17));
  struct Ack final : capture::Writer {
    void write(const capture::Event &) override {}
    void finish() override {}
  } ack;
  while (!recorder.status().writer_done) recorder.writer_step(ack);
  write_capture_outcome(writer, recorder.status(), io);
  assert(fs::exists(root / "run.capture.json"));
  {
    StreamingCaptureWriter empty(17, root / "empty", argv[1], io);
    empty.finish();
    verify_container(root / "empty/signals.mcap", {}, descriptor);
  }
  {
    StreamingCaptureWriter mixed(17, root / "mixed", argv[1], io);
    auto submission = e;
    submission.kind = capture::Kind::Submission;
    submission.host_monotonic_ns++;
    submission.position = {.5, -.125, .25, -.375, .5, -.625, .75};
    auto second = e;
    second.id = second.state_id = 2;
    second.host_monotonic_ns += 2;
    second.position[0] = std::numeric_limits<double>::infinity();
    second.nonfinite_joint_mask = 1;
    mixed.write(e);
    mixed.write(submission);
    mixed.write(second);
    mixed.finish();
    verify_container(root / "mixed/signals.mcap",
                     encode_capture_batch({e, submission, second}, 17), descriptor);
    std::ifstream quality(root / "mixed/quality.json");
    std::string report(std::istreambuf_iterator<char>{quality}, {});
    assert(report.find("\"data_valid\":false") != std::string::npos);
  }
  {
    StreamingCaptureWriter invalid(17, root / "invalid-event", argv[1], io);
    auto wrong = e; wrong.run_id = 18;
    bool rejected = false;
    try { invalid.write(wrong); } catch (const InvalidInput &) { rejected = true; }
    assert(rejected && invalid.state() == StreamingCaptureWriter::State::Failed);
    assert(invalid.staged_events().empty());
    assert(!fs::exists(root / "invalid-event/signals.mcap"));
  }
  {
    const auto existing = root / "existing";
    fs::create_directory(existing);
    std::ofstream sentinel(existing / "sentinel"); sentinel << "preserve";
    sentinel.close();
    bool rejected = false;
    try { StreamingCaptureWriter writer(17, existing, argv[1], io); }
    catch (const ExistingOutput &) { rejected = true; }
    assert(rejected && fs::exists(existing / "sentinel"));
  }
  for (const char *collision : {"signals.mcap", "quality.json"}) {
    const auto name = std::string("collision-") + collision;
    const auto out = root / name;
    StreamingCaptureWriter colliding(17, out, argv[1], io);
    colliding.write(e);
    {
      std::ofstream sentinel(out / collision);
      sentinel << "preserve";
    }
    bool failed = false;
    try { colliding.finish(); } catch (const IoError &) { failed = true; }
    assert(failed && colliding.state() == StreamingCaptureWriter::State::Failed);
    std::ifstream saved(out / collision);
    const std::string bytes(std::istreambuf_iterator<char>{saved}, {});
    assert(bytes == "preserve");
    assert(fs::exists(out / (std::string(collision) + ".partial")));
    assert(!fs::exists(fs::path(out.string() + ".capture.json")));
  }
  struct ThrowAfterWrite : FileOps {
    bool armed = false, hit = false;
    ssize_t write(int fd, const void *data, size_t size) override {
      if (armed && !hit) {
        char link[512]{};
        auto n = readlink(("/proc/self/fd/" + std::to_string(fd)).c_str(), link, 511);
        if (n > 0 && std::string(link).find("signals.mcap.partial") != std::string::npos) {
          hit = true;
          FileOps::write(fd, data, size); // bytes reached the file before throw
          throw std::runtime_error("ambiguous accepted bytes");
        }
      }
      return FileOps::write(fd, data, size);
    }
  } fault;
  StreamingCaptureWriter ambiguous(17, root / "ambiguous", argv[1], fault);
  fault.armed = true;
  bool threw = false;
  try { ambiguous.write(e); } catch (const std::runtime_error &) { threw = true; }
  assert(threw && fault.hit && ambiguous.state() == StreamingCaptureWriter::State::Failed);
  assert(ambiguous.staged_events().empty());
  assert(fs::exists(root / "ambiguous/signals.mcap.partial"));
  assert(!fs::exists(root / "ambiguous/signals.mcap"));
  fs::remove_all(root);
}
