#include "lbr_acquisition/offline_writer.hpp"
#include <iostream>
int main(int argc, char **argv) {
  try {
    if (argc != 5 || std::string(argv[1]) != "--output" ||
        std::string(argv[3]) != "--descriptor") {
      std::cerr << "usage: offline_fixture --output NEW_DIRECTORY --descriptor "
                   "FILE\n";
      return 2;
    }
    lbr_acquisition::write_fixture(argv[2], lbr_acquisition::synthetic_events(),
                                   argv[4]);
    std::cout << "Synthetic offline fixture finalized: " << argv[2] << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "offline_fixture failed: " << e.what() << '\n';
    return 1;
  }
}
