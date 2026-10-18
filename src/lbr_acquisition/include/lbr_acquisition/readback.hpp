#pragma once
#include "lbr_acquisition/offline_writer.hpp"

namespace lbr_acquisition {
// Independent, bounded linear reader. Throws IoError; never rewrites the
// source.
void verify_container(const std::filesystem::path &path,
                      const std::vector<OfflineEvent> &expected,
                      const std::string &descriptor);
} // namespace lbr_acquisition
