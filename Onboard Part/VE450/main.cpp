#include <iostream>
#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "VE450.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int main(int argc, char** argv) {
  // Initialize variables
  int functionTimeout = 1;

  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle* vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL) {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // Obtain Control Authority
  vehicle->obtainCtrlAuthority(functionTimeout);

  // Testing
  std::cout
          << "Automatic takeoff and landing test for VE450 starts..."
          <<std::endl;
  TakeoffAnyway(vehicle);
  moveByPositionOffset(vehicle,0,0,0,0);
  moveByPositionOffset(vehicle,0,0,0,0);
  moveByPositionOffset(vehicle,0,0,0,0);
  LandingAnyway(vehicle);
  std::cout
          << "Test over."
          <<std::endl;

  // Display interactive prompt
  /*std::cout
      << "| Available commands:                                            |"
      << std::endl;
  std::cout
      << "| [a] Takeoff + Landing                                |"
      << std::endl;
  std::cout
      << "| [b] Takeoff + Position Control + Landing             |"
      << std::endl;

  char inputChar;
  std::cin >> inputChar;

  switch (inputChar) {
    case 'a':
      monitoredTakeoff(vehicle);
      monitoredLanding(vehicle);
      break;
    case 'b':
      monitoredTakeoff(vehicle);
      moveByPositionOffset(vehicle, 0, 6, 6, 30);
      moveByPositionOffset(vehicle, 6, 0, -3, -30);
      moveByPositionOffset(vehicle, -6, -6, 0, 0);
      monitoredLanding(vehicle);
      break;

    default:
      break;
  }*/

  return 0;
}
