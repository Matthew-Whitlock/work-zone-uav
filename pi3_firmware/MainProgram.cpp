#include "camera_gimbal_sample.hpp"
#include <dji_telemetry.hpp>
#include "telemetry_sample.hpp"
#include  <signal.h>
#include  <stdlib.h>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

bool
subscribeToData(Vehicle* vehicle, int responseTimeout)
{
  // RTK can be detected as unavailable only for Flight controllers that don't support RTK
  bool rtkAvailable = false;
  // Counters
  int elapsedTimeInMs = 0;
  int timeToPrintInMs = 20000;

  // We will subscribe to six kinds of data:
  // 1. Flight Status at 1 Hz
  // 2. Fused Lat/Lon at 10Hz
  // 3. Fused Altitude at 10Hz
  // 4. RC Channels at 50 Hz
  // 5. Velocity at 50 Hz
  // 6. Quaternion at 200 Hz

  // Please make sure your drone is in simulation mode. You can fly the drone
  // with your RC to
  // get different values.

  // Telemetry: Verify the subscription
  ACK::ErrorCode subscribeStatus;
  subscribeStatus = vehicle->subscribe->verify(responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    return false;
  }

  // Package 0: Subscribe to flight status at freq 1 Hz
  int       pkgIndex        = 0;
  int       freq            = 1;
  TopicName topicList1Hz[]  = { TOPIC_STATUS_FLIGHT };
  int       numTopic        = sizeof(topicList1Hz) / sizeof(topicList1Hz[0]);
  bool      enableTimestamp = false;

  bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList1Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }
  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup before return
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    return false;
  }

  // Package 1: Subscribe to Lat/Lon, and Alt at freq 10 Hz
  pkgIndex                  = 1;
  freq                      = 10;
  TopicName topicList10Hz[] = { TOPIC_GPS_FUSED, TOPIC_ALTITUDE_FUSIONED};
  numTopic                  = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
  enableTimestamp           = false;

  pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }
  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup before return
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    return false;
  }

  // Package 2: Subscribe to RC Channel and Velocity at freq 50 Hz
  pkgIndex                  = 2;
  freq                      = 50;
  TopicName topicList50Hz[] = { TOPIC_RC, TOPIC_VELOCITY };
  numTopic                  = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
  enableTimestamp           = false;

  pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }
  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup before return
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    return false;
  }

  // Package 3: Subscribe to Quaternion at freq 200 Hz.
  pkgIndex                   = 3;
  freq                       = 200;
  TopicName topicList200Hz[] = { TOPIC_QUATERNION };
  numTopic        = sizeof(topicList200Hz) / sizeof(topicList200Hz[0]);
  enableTimestamp = false;

  pkgStatus = vehicle->subscribe->initPackageFromTopicList(
          pkgIndex, numTopic, topicList200Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
      return pkgStatus;
  }
  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
      ACK::getErrorCodeMessage(subscribeStatus, __func__);
      // Cleanup before return
      vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
      return false;
  }

  // Package 4: Subscribe to RTK at freq 5 Hz.
  pkgIndex                   = 4;
  freq                       = 5;
  TopicName topicListRTK5Hz[] = {TOPIC_RTK_POSITION, TOPIC_RTK_YAW_INFO,
                                  TOPIC_RTK_POSITION_INFO, TOPIC_RTK_VELOCITY,
                                  TOPIC_RTK_YAW};
  numTopic        = sizeof(topicListRTK5Hz) / sizeof(topicListRTK5Hz[0]);
  enableTimestamp = false;

  pkgStatus = vehicle->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicListRTK5Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }
  else {
    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
    if(subscribeStatus.data == ErrorCode::SubscribeACK::SOURCE_DEVICE_OFFLINE)
    {
      std::cout << "RTK Not Available" << "\n";
      rtkAvailable = false;
    }
    else
    {
      rtkAvailable = true;
      if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        // Cleanup before return
        vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
        return false;
      }
    }
  }

  // Wait for the data to start coming in.
  sleep(1);

  // Get all the data once before the loop to initialize vars
  TypeMap<TOPIC_STATUS_FLIGHT>::type     flightStatus;
  TypeMap<TOPIC_GPS_FUSED>::type         latLon;
  TypeMap<TOPIC_ALTITUDE_FUSIONED>::type altitude;
  TypeMap<TOPIC_RC>::type                rc;
  TypeMap<TOPIC_VELOCITY>::type          velocity;
  TypeMap<TOPIC_QUATERNION>::type        quaternion;
  TypeMap<TOPIC_RTK_POSITION>::type      rtk;
  TypeMap<TOPIC_RTK_POSITION_INFO>::type rtk_pos_info;
  TypeMap<TOPIC_RTK_VELOCITY>::type      rtk_velocity;
  TypeMap<TOPIC_RTK_YAW>::type           rtk_yaw;
  TypeMap<TOPIC_RTK_YAW_INFO>::type      rtk_yaw_info;

  // Print in a loop for 2 sec
  while (elapsedTimeInMs < timeToPrintInMs)
  {
    flightStatus = vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>();
    latLon       = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    altitude     = vehicle->subscribe->getValue<TOPIC_ALTITUDE_FUSIONED>();
    rc           = vehicle->subscribe->getValue<TOPIC_RC>();
    velocity     = vehicle->subscribe->getValue<TOPIC_VELOCITY>();
    quaternion   = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
    if(rtkAvailable) {
      rtk = vehicle->subscribe->getValue<TOPIC_RTK_POSITION>();
      rtk_pos_info = vehicle->subscribe->getValue<TOPIC_RTK_POSITION_INFO>();
      rtk_velocity = vehicle->subscribe->getValue<TOPIC_RTK_VELOCITY>();
      rtk_yaw = vehicle->subscribe->getValue<TOPIC_RTK_YAW>();
      rtk_yaw_info = vehicle->subscribe->getValue<TOPIC_RTK_YAW_INFO>();
    }
    std::cout << "Counter = " << elapsedTimeInMs << ":\n";
    std::cout << "-------\n";
    std::cout << "Flight Status                         = " << (int)flightStatus
              << "\n";
    std::cout << "Position              (LLA)           = " << latLon.latitude
              << ", " << latLon.longitude << ", " << altitude << "\n";
    std::cout << "RC Commands           (r/p/y/thr)     = " << rc.roll << ", "
              << rc.pitch << ", " << rc.yaw << ", " << rc.throttle << "\n";
    std::cout << "Velocity              (vx,vy,vz)      = " << velocity.data.x
              << ", " << velocity.data.y << ", " << velocity.data.z << "\n";
    std::cout << "Attitude Quaternion   (w,x,y,z)       = " << quaternion.q0
              << ", " << quaternion.q1 << ", " << quaternion.q2 << ", "
              << quaternion.q3 << "\n";
    if(rtkAvailable) {
      std::cout << "RTK if available   (lat/long/alt/velocity_x/velocity_y/velocity_z/yaw/yaw_info/pos_info) ="
                << rtk.latitude << "," << rtk.longitude << "," << rtk.HFSL << "," << rtk_velocity.x << ","
                << rtk_velocity.y
                << "," << rtk_velocity.z << "," << rtk_yaw << "," << rtk_yaw_info << rtk_pos_info << "\n";
    }
    std::cout << "-------\n\n";
    usleep(5000);
    elapsedTimeInMs += 5;
  }

  std::cout << "Done printing!\n";
  vehicle->subscribe->removePackage(0, responseTimeout);
  vehicle->subscribe->removePackage(1, responseTimeout);
  vehicle->subscribe->removePackage(2, responseTimeout);
  vehicle->subscribe->removePackage(3, responseTimeout);
  vehicle->subscribe->removePackage(4, responseTimeout);

  return true;
}

bool
gimbalCameraControl(Vehicle* vehicle)
{
  if(!vehicle->gimbal)
  {
    DERROR("Gimbal object does not exist.\n");
    return false;
  }

  int responseTimeout = 0;

  GimbalContainer              gimbal;
  RotationAngle                currentAngle;
  DJI::OSDK::Gimbal::SpeedData gimbalSpeed;
  int                          pkgIndex;

  /*
   * Subscribe to gimbal data not supported in MAtrice 100
   */

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    // Telemetry: Verify the subscription
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = vehicle->subscribe->verify(responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, __func__);
      return false;
    }

    // Telemetry: Subscribe to gimbal status and gimbal angle at freq 10 Hz
    pkgIndex                  = 0;
    int       freq            = 10;
    TopicName topicList10Hz[] = { TOPIC_GIMBAL_ANGLES, TOPIC_GIMBAL_STATUS };
    int       numTopic = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
    bool      enableTimestamp = false;

    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
    if (!(pkgStatus))
    {
      return pkgStatus;
    }
    subscribeStatus =
      vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, __func__);
      // Cleanup before return
      vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
      return false;
    }
  }

  sleep(1);

  std::cout
    << "Please note that the gimbal yaw angle you see in the telemetry is "
       "w.r.t absolute North"
       ", and the accuracy depends on your magnetometer calibration.\n\n";

  

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    currentAngle.roll  = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().y;
    currentAngle.pitch = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().x;
    currentAngle.yaw   = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().z;
  }
  else
  {
    currentAngle.roll  = vehicle->broadcast->getGimbal().roll;
    currentAngle.pitch = vehicle->broadcast->getGimbal().pitch;
    currentAngle.yaw   = vehicle->broadcast->getGimbal().yaw;
  }

  displayResult(&currentAngle);


  // Cleanup and exit gimbal sample
  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    ACK::ErrorCode ack =
      vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    if (ACK::getError(ack))
    {
      std::cout
        << "Error unsubscribing; please restart the drone/FC to get back "
           "to a clean state.\n";
    }
  }

  return true;
}

void
doSetGimbalAngle(Vehicle* vehicle, GimbalContainer* gimbal)
{
  DJI::OSDK::Gimbal::AngleData gimbalAngle;
  gimbalAngle.roll     = gimbal->roll;
  gimbalAngle.pitch    = gimbal->pitch;
  gimbalAngle.yaw      = gimbal->yaw;
  gimbalAngle.duration = gimbal->duration;
  gimbalAngle.mode |= 0;
  gimbalAngle.mode |= gimbal->isAbsolute;
  gimbalAngle.mode |= gimbal->yaw_cmd_ignore << 1;
  gimbalAngle.mode |= gimbal->roll_cmd_ignore << 2;
  gimbalAngle.mode |= gimbal->pitch_cmd_ignore << 3;

  vehicle->gimbal->setAngle(&gimbalAngle);
  // Give time for gimbal to sync
  sleep(4);
}

void
displayResult(RotationAngle* currentAngle)
{
  std::cout << "New Gimbal rotation angle is [";
  std::cout << currentAngle->roll << " ";
  std::cout << currentAngle->pitch << " ";
  std::cout << currentAngle->yaw;
  std::cout << "]\n\n";
}
