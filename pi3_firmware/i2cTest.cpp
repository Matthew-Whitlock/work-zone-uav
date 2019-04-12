
#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <stdio.h>
#include <string>
#include <errno.h>
#include <termios.h>
#define LW20_API_IMPLEMENTATION
#include "lw20api.h"

	int file_i2c;
	int length;
	unsigned char buffer[60] = {0};
	std::string toSend;
	char tempBuf[60]={0};
	
	int
	set_interface_attribs (int fd, int speed, int parity)
	{
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

int main(void)
{	
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
	
	
	//Open Serial
	int serial_port = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
	
	set_interface_attribs(serial_port,B9600,1);
	
	
	
	
	
	//----- WRITE BYTES -----
	
	buffer[0]=0;
	buffer[1]=81;
	
	length = 2;			//<<< Number of bytes to write
	if(write(file_i2c, buffer, length) != length)		//write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
	{
		// ERROR HANDLING: i2c transaction failed
		printf("Failed to write to the i2c bus.\n");
	}
	usleep(750000);		
	
	//----- READ BYTES -----
	length = 2;			//<<< Number of bytes to read
	if(read(file_i2c, buffer, length) != length)		//read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
	{
		//ERROR HANDLING: i2c transaction failed
		printf("Failed to read from the i2c bus.\n");
	}
	else
	{
		printf("Distance: %u\n", buffer[1]);
	}
	while(true){
		usleep(75000);
		length = 2;			//<<< Number of bytes to read
		if(read(file_i2c, buffer, length) != length)		//read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
		{
			//ERROR HANDLING: i2c transaction failed
			printf("Failed to read from the i2c bus.\n");
		}
		else
		{
			toSend=sprintf(tempBuf,"Distance: %d\r\n", buffer[1]);
			printf("Distance: %d\n", buffer[1]);
			write(serial_port, &toSend, 60);
			
		}
	}
	
}

