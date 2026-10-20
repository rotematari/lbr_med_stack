#include "lbr_acquisition/streaming_capture_writer.hpp"
#include "lbr_acquisition/capture_encoding.hpp"

namespace lbr_acquisition {
namespace {
uint64_t valid_run_id(uint64_t run_id) {
  if (!run_id) throw InvalidInput("streaming writer requires nonzero run ID");
  return run_id;
}
}
StreamingCaptureWriter::StreamingCaptureWriter(uint64_t run_id,
    std::filesystem::path directory, std::filesystem::path descriptor, FileOps &io,
    CaptureSource source, CaptureLimits limits)
    : run_id_(valid_run_id(run_id)), source_(source),
      directory_(std::move(directory)), limits_(checked_limits(limits)),
      writer_(directory_, std::move(descriptor), io, source_, limits_, true) {}
void StreamingCaptureWriter::write(const capture::Event &event) {
  if (state_ != State::Accepting) throw InvalidInput("streaming writer closed");
  try {
    if (summary_.events >= limits_.events)
      throw InvalidInput("streaming event limit");
    const auto encoded = encode_capture_event(event, run_id_, latest_state_);
    auto next = summary_;
    next.observe(encoded); // Prepare evidence before the potentially ambiguous write.
    writer_.append(encoded, true);
    summary_ = next; // Fixed-size, no allocation/throw after acknowledgement.
    if (event.kind == capture::Kind::State) latest_state_ = event.id;
  } catch (...) {
    state_ = State::Failed;
    throw;
  }
}
void StreamingCaptureWriter::finish() {
  if (state_ != State::Accepting) throw InvalidInput("streaming writer closed");
  try {
    writer_.finish_capture(summary_);
    state_ = State::Finalized;
  } catch (...) {
    state_ = State::Failed;
    throw;
  }
}
} // namespace lbr_acquisition
