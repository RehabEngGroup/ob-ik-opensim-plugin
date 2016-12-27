#include <OpenSim/OpenSim.h>
#include "OpenSim/Tools/InverseKinematicsExtendedTool.h"

int main(int argc, char* argv[]) {

  if (argc < 2) {
    std::cout << "Please provide a setup file" << std::endl;
    return 0;
  }

  OpenSim::InverseKinematicsExtendedTool ik(argv[1]);

  // start timing
  std::clock_t startTime = std::clock();

  // RUN
  ik.run();

  std::cout << "IK compute time = " << 1.e3*(std::clock() - startTime) / CLOCKS_PER_SEC << "ms\n";
  return 0;
}
