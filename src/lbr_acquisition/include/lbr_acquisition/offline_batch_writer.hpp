#pragma once
#include "lbr_acquisition/offline_writer.hpp"
#include "lbr_acquisition/recorder.hpp"

namespace lbr_acquisition {
// Synthetic-only, single-use batch adapter. All methods/getters require serial
// access on the writer side (or after it is quiescent). FileOps must outlive
// this object. write acknowledges RAM staging, NOT persistence. No files until
// finish; destruction never finalizes. This is not an operational/live dataset
// path.
class OfflineBatchWriter final : public capture::Writer {
public:
  enum class State { Accepting, Finalized, Failed };
  OfflineBatchWriter(uint64_t run_id, std::filesystem::path new_run_dir,
                     std::filesystem::path descriptor_file, FileOps &io);
  OfflineBatchWriter(const OfflineBatchWriter &) = delete;
  OfflineBatchWriter &operator=(const OfflineBatchWriter &) = delete;
  OfflineBatchWriter(OfflineBatchWriter &&) = delete;
  OfflineBatchWriter &operator=(OfflineBatchWriter &&) = delete;
  void write(const capture::Event &) override;
  void finish() override;
  State state() const noexcept { return state_; }
  uint64_t run_id() const noexcept { return run_id_; }
  const std::filesystem::path &output_directory() const noexcept { return directory_; }
  // Only acknowledged writes; retained on failure, valid until destruction.
  const std::vector<capture::Event> &staged_events() const noexcept {
    return events_;
  }

private:
  void require_accepting() const;
  const uint64_t run_id_;
  const std::filesystem::path directory_, descriptor_;
  FileOps &io_;
  std::vector<capture::Event> events_;
  State state_ = State::Accepting;
};
} // namespace lbr_acquisition
