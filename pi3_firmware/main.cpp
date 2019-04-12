
#include "camera_gimbal_sample.hpp"
#include "telemetry_sample.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int
main(int argc, char** argv)
{

  // Setup the OSDK: Read config file, create vehicle, activate.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }
  subscribeToData(vehicle);
  gimbalCameraControl(vehicle);
  
  

  return 0;
}
