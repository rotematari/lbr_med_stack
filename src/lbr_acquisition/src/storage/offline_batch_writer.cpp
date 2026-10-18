#include "lbr_acquisition/offline_batch_writer.hpp"
#include "lbr_acquisition/capture_encoding.hpp"
#include <utility>

namespace lbr_acquisition {
OfflineBatchWriter::OfflineBatchWriter(uint64_t run_id,
                                       std::filesystem::path directory,
                                       std::filesystem::path descriptor,
                                       FileOps &io)
    : run_id_(run_id), directory_(std::move(directory)),
      descriptor_(std::move(descriptor)), io_(io) {
  if (!run_id_)
    throw InvalidInput("batch writer requires nonzero run ID");
  events_.reserve(10000);
}
void OfflineBatchWriter::require_accepting() const {
  if (state_ != State::Accepting)
    throw InvalidInput("batch writer is already finalized or failed");
}
void OfflineBatchWriter::write(const capture::Event &event) {
  require_accepting();
  try {
    if (event.run_id != run_id_ || events_.size() >= 10000)
      throw InvalidInput("batch writer run mismatch or event limit");
    events_.push_back(event);
  } catch (...) {
    state_ = State::Failed;
    throw;
  }
}
void OfflineBatchWriter::finish() {
  require_accepting();
  try {
    const auto encoded = encode_capture_batch(events_, run_id_);
    write_fixture(directory_, encoded, descriptor_, io_);
    state_ = State::Finalized;
  } catch (...) {
    state_ = State::Failed;
    throw;
  }
}
} // namespace lbr_acquisition
