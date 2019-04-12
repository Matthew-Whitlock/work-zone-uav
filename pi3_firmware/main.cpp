#include <dji_telemetry.hpp>
#include "telemetry_sample.hpp"
#include  <signal.h>
#include  <stdlib.h>

#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <stdio.h>
#include <sstream>
#include <errno.h>
#include <termios.h>
#define LW20_API_IMPLEMENTATION
#include "lw20api.h"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
	
int file_i2c;
int length;
unsigned char lidar_buffer[60] = {0};
std::stringstream toSend;
	
int set_interface_attribs (int fd, int speed, int parity) {
   struct termios tty;
   //memset (&tty, 0, sizeof tty);
   if (tcgetattr (fd, &tty) != 0)
   {
         //error_message ("error %d from tcgetattr", errno);
         return -1;
   }

   cfsetospeed (&tty, speed);
   cfsetispeed (&tty, speed);

   tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
   // disable IGNBRK for mismatched speed tests; otherwise receive break
   // as \000 chars
   tty.c_iflag &= ~IGNBRK;         // disable break processing
   tty.c_lflag = 0;                // no signaling chars, no echo,
                        // no canonical processing
   tty.c_oflag = 0;                // no remapping, no delays
   tty.c_cc[VMIN]  = 0;            // read doesn't block
   tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

   tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

   tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                           // enable reading
   tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
   tty.c_cflag |= parity;
   tty.c_cflag &= ~CSTOPB;
   tty.c_cflag &= ~CRTSCTS;

   if (tcsetattr (fd, TCSANOW, &tty) != 0)
   {
         //error_message ("error %d from tcsetattr", errno);
         return -1;
   }
   return 0;
}

int main(int argc, char** argv) {

   //Open Serial
   int serial_port = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
   set_interface_attribs(serial_port,B9600,1);

	//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";
	if((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
		return 0;
	}
	
	int addr = 0x66;          //<<<<<The I2C address of the slave
	if(ioctl(file_i2c, I2C_SLAVE, addr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
		//ERROR HANDLING; you can check errno to see what went wrong
		return 0;
	}
	


  // Setup the OSDK: Read config file, create vehicle, activate.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }
  
  if(!vehicle->gimbal)
  {
    DERROR("Gimbal object does not exist.\n");
    return false;
  }
  
  int responseTimeout = 1000;
  // Counters
  int elapsedTimeInMs = 0;
  int timeToPrintInMs = 20000;

  // We will subscribe to six kinds of data:
  // 2. Fused Lat/Lon at 10Hz
  // 3. Fused Altitude at 10Hz
  // 5. Velocity at 10 Hz
  // 6. Quaternion at 10 Hz

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
  int freq                      = 10;  
  TopicName topicList10Hz[] = { TOPIC_STATUS_FLIGHT, TOPIC_GPS_FUSED, TOPIC_ALTITUDE_FUSIONED, TOPIC_VELOCITY, TOPIC_QUATERNION, TOPIC_GIMBAL_ANGLES, TOPIC_GIMBAL_STATUS};
  int numTopic                  = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
  bool enableTimestamp           = false;

  bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
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

  // Wait for the data to start coming in.
  sleep(1);

  // Get all the data once before the loop to initialize vars
  TypeMap<TOPIC_STATUS_FLIGHT>::type     flightStatus;
  TypeMap<TOPIC_GPS_FUSED>::type         latLon;
  TypeMap<TOPIC_ALTITUDE_FUSIONED>::type altitude;
  TypeMap<TOPIC_VELOCITY>::type          velocity;
  TypeMap<TOPIC_QUATERNION>::type        quaternion;
  int lidar_height = 0;
	
  //----- WRITE BYTES to control lidar-----	
  lidar_buffer[0]=0;
  lidar_buffer[1]=81;
	
  length = 2;			//<<< Number of bytes to write
  if(write(file_i2c, lidar_buffer, length) != length)		//write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
  {
    // ERROR HANDLING: i2c transaction failed
    printf("Failed to write to the i2c bus.\n");
  }
  usleep(5000);
  
  // Print in a loop for 2 sec
  while (elapsedTimeInMs < timeToPrintInMs) {
	 //read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
	 if(read(file_i2c, lidar_buffer, length) != length){
      toSend << "Failed to get LIDAR height\r\n\r\n";
    } else {
      lidar_height = lidar_buffer[1];
    }

    flightStatus = vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>();
    latLon       = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    altitude     = vehicle->subscribe->getValue<TOPIC_ALTITUDE_FUSIONED>();
    velocity     = vehicle->subscribe->getValue<TOPIC_VELOCITY>();
    quaternion   = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
    
    //Get gimbal (camera) angle information.
    DJI::OSDK::float32_t pitch = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().x;
    DJI::OSDK::float32_t roll = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().y;
    DJI::OSDK::float32_t yaw = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().z;
			
    
    toSend << "Flight Status                         = " << (int)flightStatus
              << "\r\n";
    toSend << "Position              (LLA)           = " << latLon.latitude
              << ", " << latLon.longitude << ", " << altitude << "\r\n";
    toSend << "Velocity              (vx,vy,vz)      = " << velocity.data.x
              << ", " << velocity.data.y << ", " << velocity.data.z << "\r\n";
    toSend << "Attitude Quaternion   (w,x,y,z)       = " << quaternion.q0
              << ", " << quaternion.q1 << ", " << quaternion.q2 << ", "
              << quaternion.q3 << "\r\n";
    toSend << "Gimbal              (roll,pitch,yaw)      = " << roll
              << ", " << pitch  << ", " << yaw << "\r\n";
    toSend << "Lidar height              (cm)      = " << lidar_height << "\r\n";
    
    toSend << "-------\r\n\r\n";
	 printf("Sending: %s", &(toSend.str().front()));
	 write(serial_port, &(toSend.str().front()), toSend.str().size());
    toSend.str("");
    
    usleep(5000);
    elapsedTimeInMs += 5;
  }

  std::cout << "Done printing!\n";
  vehicle->subscribe->removePackage(0, responseTimeout);

  return 0;
}
