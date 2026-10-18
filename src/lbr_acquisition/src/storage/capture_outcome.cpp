#include "lbr_acquisition/capture_outcome.hpp"
#include "lbr_acquisition/capture_encoding.hpp"
#include "lbr_acquisition/readback.hpp"
#include <array>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <linux/fs.h>
#include <openssl/sha.h>
#include <sstream>
#include <sys/syscall.h>
#include <unistd.h>

namespace lbr_acquisition {
namespace {
namespace fs = std::filesystem;
constexpr size_t json_limit = 65536, mcap_limit = 16 * 1024 * 1024;

const char *phase_name(capture::Phase p) {
  switch (p) {
  case capture::Phase::Complete: return "Complete";
  case capture::Phase::Failed: return "Failed";
  default: throw InvalidInput("outcome requires terminal Recorder phase");
  }
}
const char *reason_name(capture::Reason r) {
  switch (r) {
  case capture::Reason::Manual: return "Manual";
  case capture::Reason::Duration: return "Duration";
  case capture::Reason::NoData: return "NoData";
  case capture::Reason::Overflow: return "Overflow";
  case capture::Reason::InvalidEvent: return "InvalidEvent";
  case capture::Reason::EventLimit: return "EventLimit";
  case capture::Reason::SourceUnavailable: return "SourceUnavailable";
  case capture::Reason::ClockRegression: return "ClockRegression";
  case capture::Reason::WriterFailure: return "WriterFailure";
  default: throw InvalidInput("invalid terminal Recorder reason");
  }
}
const char *sink_name(OfflineBatchWriter::State s) {
  switch (s) {
  case OfflineBatchWriter::State::Finalized: return "Finalized";
  case OfflineBatchWriter::State::Failed: return "Failed";
  default: throw InvalidInput("outcome requires terminal sink state");
  }
}
template <class Sink> void validate(const Sink &sink, const capture::Status &s) {
  if (!sink.run_id() || !s.writer_done || s.writer_in_call)
    throw InvalidInput("outcome requires quiescent terminal Recorder");
  phase_name(s.phase);
  reason_name(s.reason);
  sink_name(sink.state());
  constexpr uint32_t material_mask =
      ((1u << (static_cast<unsigned>(capture::Reason::WriterFailure) + 1)) - 1) &
      ~((1u << (static_cast<unsigned>(capture::Reason::Duration) + 1)) - 1);
  if (s.failure_flags & ~material_mask)
    throw InvalidInput("invalid failure flags");
  if (s.phase == capture::Phase::Complete) {
    if (s.failure_flags || sink.state() != OfflineBatchWriter::State::Finalized ||
        (s.reason != capture::Reason::Manual &&
         s.reason != capture::Reason::Duration))
      throw InvalidInput("inconsistent complete outcome");
  } else {
    if (!s.failure_flags ||
        (static_cast<unsigned>(s.reason) >
             static_cast<unsigned>(capture::Reason::Duration) &&
         !(s.failure_flags & (1u << static_cast<unsigned>(s.reason)))))
      throw InvalidInput("inconsistent failed outcome");
  }
  if (sink.state() == OfflineBatchWriter::State::Failed &&
      !(s.failure_flags & (1u << static_cast<unsigned>(capture::Reason::WriterFailure))))
    throw InvalidInput("failed sink requires WriterFailure flag");
  if (s.attempted < s.enqueued || s.attempted - s.enqueued != s.rejected ||
      s.enqueued < s.written || s.enqueued - s.written < s.abandoned ||
      s.enqueued - s.written - s.abandoned != s.write_uncertain ||
      s.enqueued > 10000 || s.written > 10000 ||
      sink.staged_events().size() != s.written ||
      (!s.has_first && (s.first_state_ns || s.last_state_ns)) ||
      (s.has_first && s.last_state_ns < s.first_state_ns))
    throw InvalidInput("inconsistent terminal counters or state clock");
  if (sink.state() == OfflineBatchWriter::State::Finalized) {
    if (s.enqueued != s.written || s.abandoned || s.write_uncertain)
      throw InvalidInput("finalized sink has unwritten events");
  }
}
template <class Sink> void validate_staged_clocks(const Sink &sink,
                            const capture::Status &s) {
  bool found = false;
  uint64_t first = 0, last = 0;
  for (const auto &e : sink.staged_events()) {
    if (e.kind != capture::Kind::State) continue;
    if (!found) first = e.host_monotonic_ns;
    last = e.host_monotonic_ns;
    found = true;
  }
  if (found != s.has_first ||
      (found && (first != s.first_state_ns || last != s.last_state_ns)))
    throw InvalidInput("Recorder State clocks disagree with staged events");
}
std::string read_bounded(const fs::path &path, size_t limit) {
  std::ifstream f(path, std::ios::binary | std::ios::ate);
  if (!f) throw IoError("cannot read artifact: " + path.string());
  const auto length = f.tellg();
  if (length < 0 || static_cast<uint64_t>(length) > limit)
    throw IoError("artifact exceeds profile limit");
  std::string bytes(static_cast<size_t>(length), '\0');
  f.seekg(0);
  if (!bytes.empty()) f.read(bytes.data(), bytes.size());
  if (!f) throw IoError("artifact read failed");
  return bytes;
}
std::string sha256(const std::string &bytes) {
  unsigned char hash[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char *>(bytes.data()), bytes.size(), hash);
  std::ostringstream o;
  for (auto c : hash)
    o << std::hex << std::setw(2) << std::setfill('0') << unsigned(c);
  return o.str();
}
bool regular_or_absent(const fs::path &p) {
  std::error_code ec;
  auto st = fs::symlink_status(p, ec);
  if (ec == std::errc::no_such_file_or_directory) return false;
  if (ec) throw IoError("artifact stat failed: " + ec.message());
  if (fs::is_symlink(st) || !fs::is_regular_file(st))
    throw InvalidInput("artifact is not a regular file");
  return true;
}
std::string artifact_json(const fs::path &p, size_t limit) {
  if (!regular_or_absent(p)) return "null";
  const auto bytes = read_bounded(p, limit);
  return "{\"size_bytes\":\"" + std::to_string(bytes.size()) +
         "\",\"sha256\":\"" + sha256(bytes) + "\"}";
}
void write_exact(int fd, const std::string &bytes, FileOps &io) {
  const char *p = bytes.data();
  size_t remaining = bytes.size();
  while (remaining) {
    const auto n = io.write(fd, p, remaining);
    if (n < 0 && errno == EINTR) continue;
    if (n < 0) throw IoError("outcome write failed: " + std::string(std::strerror(errno)));
    if (n == 0 || static_cast<size_t>(n) > remaining)
      throw IoError("outcome write made no progress");
    p += n; remaining -= n;
  }
}
} // namespace

template <class Sink> void write_capture_outcome_impl(const Sink &sink,
                           const capture::Status &s, FileOps &io) {
  validate(sink, s);
  const auto &dir = sink.output_directory();
  const auto leaf = dir.filename().string();
  if (leaf.empty() || leaf == "." || leaf == "..")
    throw InvalidInput("invalid outcome directory filename");
  {
    std::error_code ec;
    const auto st = fs::symlink_status(dir, ec);
    if (ec != std::errc::no_such_file_or_directory && ec)
      throw IoError("run directory stat failed: " + ec.message());
    if (!ec && (fs::is_symlink(st) || !fs::is_directory(st)))
      throw InvalidInput("run path is not a real directory");
  }
  const fs::path final = dir.parent_path() / (leaf + ".capture.json");
  const fs::path partial = final.string() + ".partial";
  // Refuse symlinks as well as existing regular paths, without following them.
  for (const auto &p : {final, partial}) {
    std::error_code ec;
    const auto st = fs::symlink_status(p, ec);
    if (ec != std::errc::no_such_file_or_directory && ec)
      throw IoError("outcome path stat failed: " + ec.message());
    if (!ec && fs::exists(st)) throw InvalidInput("outcome path already exists");
  }
  const auto manifest = artifact_json(dir / "manifest.json", json_limit);
  const auto quality = artifact_json(dir / "quality.json", json_limit);
  const auto signals = artifact_json(dir / "signals.mcap", mcap_limit);
  if (sink.state() == OfflineBatchWriter::State::Finalized &&
      (manifest == "null" || quality == "null" || signals == "null"))
    throw InvalidInput("finalized sink lacks complete artifacts");
  // A failed sink may still have published a final MCAP before quality close
  // failed. With no final MCAP, Recorder may have seen a State that the sink
  // never acknowledged, so staged State clocks cannot be compared.
  if (sink.state() == OfflineBatchWriter::State::Finalized || signals != "null")
    validate_staged_clocks(sink, s);
  if (signals != "null") {
    if (s.enqueued != s.written || s.abandoned || s.write_uncertain)
      throw InvalidInput("final signal file disagrees with Recorder accounting");
    const auto desc = read_bounded(LBR_BUILD_DESCRIPTOR, json_limit);
    verify_container(dir / "signals.mcap",
                     encode_capture_batch(sink.staged_events(), sink.run_id()), desc);
  }
  std::string json = "{\"schema_version\":1,\"profile\":\"offline-capture-outcome-v1\","
                     "\"source\":\"synthetic\",\"capture_run_id\":\"" +
                     std::to_string(sink.run_id()) + "\",\"phase\":\"" +
                     phase_name(s.phase) + "\",\"reason\":\"" + reason_name(s.reason) +
                     "\",\"failure_flags\":" + std::to_string(s.failure_flags) +
                     ",\"writer_done\":true,\"writer_in_call\":false,\"has_first\":" +
                     (s.has_first ? "true" : "false") +
                     ",\"first_state_ns\":\"" + std::to_string(s.first_state_ns) +
                     "\",\"last_state_ns\":\"" + std::to_string(s.last_state_ns) +
                     "\",\"attempted\":\"" + std::to_string(s.attempted) +
                     "\",\"enqueued\":\"" + std::to_string(s.enqueued) +
                     "\",\"rejected\":\"" + std::to_string(s.rejected) +
                     "\",\"written\":\"" + std::to_string(s.written) +
                     "\",\"abandoned\":\"" + std::to_string(s.abandoned) +
                     "\",\"write_uncertain\":\"" + std::to_string(s.write_uncertain) +
                     "\",\"sink_state\":\"" + sink_name(sink.state()) +
                     "\",\"power_loss_durability\":false,\"artifacts\":{"
                     "\"manifest.json\":" + manifest + ",\"quality.json\":" + quality +
                     ",\"signals.mcap\":" + signals + "}}\n";
  if (json.size() > json_limit) throw InvalidInput("outcome metadata exceeds bound");
  const int fd = ::open(partial.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
  if (fd < 0) throw IoError("outcome open failed: " + std::string(std::strerror(errno)));
  try {
    write_exact(fd, json, io);
  } catch (...) {
    const int saved = errno;
    io.close(fd); // exactly once; preserve the first error and partial file
    errno = saved;
    throw;
  }
  if (io.close(fd) < 0)
    throw IoError("outcome close failed: " + std::string(std::strerror(errno)));
  if (::syscall(SYS_renameat2, AT_FDCWD, partial.c_str(), AT_FDCWD,
                final.c_str(), RENAME_NOREPLACE) < 0)
    throw IoError("outcome no-clobber publish failed: " +
                  std::string(std::strerror(errno)));
}
void write_capture_outcome(const OfflineBatchWriter &sink,
                           const capture::Status &s, FileOps &io) {
  write_capture_outcome_impl(sink, s, io);
}
void write_capture_outcome(const StreamingCaptureWriter &sink,
                           const capture::Status &s, FileOps &io) {
  write_capture_outcome_impl(sink, s, io);
}
} // namespace lbr_acquisition
