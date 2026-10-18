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
    std::filesystem::path directory, std::filesystem::path descriptor, FileOps &io)
    : run_id_(valid_run_id(run_id)), directory_(std::move(directory)),
      writer_(directory_, std::move(descriptor), io) {
  acknowledged_.reserve(10000);
}
void StreamingCaptureWriter::write(const capture::Event &event) {
  if (state_ != State::Accepting) throw InvalidInput("streaming writer closed");
  try {
    if (acknowledged_.size() >= 10000)
      throw InvalidInput("streaming event limit");
    const auto encoded = encode_capture_event(event, run_id_, latest_state_);
    writer_.append(encoded, true);
    acknowledged_.push_back(event); // reserved, trivially copyable: no throw
    if (event.kind == capture::Kind::State) latest_state_ = event.id;
  } catch (...) {
    state_ = State::Failed;
    throw;
  }
}
void StreamingCaptureWriter::finish() {
  if (state_ != State::Accepting) throw InvalidInput("streaming writer closed");
  try {
    writer_.finish(encode_capture_batch(acknowledged_, run_id_));
    state_ = State::Finalized;
  } catch (...) {
    state_ = State::Failed;
    throw;
  }
}
} // namespace lbr_acquisition
