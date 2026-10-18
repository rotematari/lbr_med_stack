#pragma once
#include "acquisition_v1.pb.h"
#include <filesystem>
#include <stdexcept>
#include <string>
#include <sys/types.h>
#include <variant>
#include <vector>

namespace lbr_acquisition {
using OfflineEvent =
    std::variant<lbr::acquisition::v1::State, lbr::acquisition::v1::Submission>;
struct InvalidInput : std::runtime_error {
  using std::runtime_error::runtime_error;
};
struct ExistingOutput : std::runtime_error {
  using std::runtime_error::runtime_error;
};
struct IoError : std::runtime_error {
  using std::runtime_error::runtime_error;
};
// Injectable POSIX boundary; implementations must follow write/close errno
// semantics. No buffering here: successful write means bytes accepted by the
// kernel, not durable storage.
struct FileOps {
  virtual ~FileOps() = default;
  virtual ssize_t write(int fd, const void *data, size_t size);
  virtual int close(int fd);
};
std::vector<OfflineEvent> synthetic_events();
std::vector<std::string>
validate_events(const std::vector<OfflineEvent> &events);
void write_fixture(const std::filesystem::path &new_run_dir,
                   const std::vector<OfflineEvent> &events,
                   const std::filesystem::path &descriptor_file);
// Same pipeline with injected file I/O, including manifest and diagnostic
// writes.
void write_fixture(const std::filesystem::path &new_run_dir,
                   const std::vector<OfflineEvent> &events,
                   const std::filesystem::path &descriptor_file, FileOps &io);
} // namespace lbr_acquisition
