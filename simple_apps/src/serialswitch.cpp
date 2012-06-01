/*
 * serialswitch.cpp
 *
 * Created on: Oct 6, 2011
 * Author: pmajcher
 */
#include <ros/ros.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <cstring>
#include <iostream>

#include "serialswitch.hpp"

using namespace std;

SerialSwitch::SerialSwitch(const std::string& port, int baud) {
	connected = false;

	fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd >= 0) {
		tcgetattr(fd, &oldtio);

		// set up new settings
		struct termios newtio;
		memset(&newtio, 0, sizeof(newtio));
		newtio.c_cflag = CBAUD | CS8 | CLOCAL | CREAD;
		newtio.c_iflag = INPCK; //IGNPAR;
		newtio.c_oflag = 0;
		newtio.c_lflag = 0;
		if (cfsetispeed(&newtio, baud) < 0 || cfsetospeed(&newtio, baud) < 0) {
			fprintf(stderr, "Failed to set serial baud rate: %d\n", baud);
			tcsetattr(fd, TCSANOW, &oldtio);
			close(fd);
			fd = -1;
			return;
		}
		// activate new settings
		tcflush(fd, TCIFLUSH);
		tcsetattr(fd, TCSANOW, &newtio);
		connected = true;
	}

	_dump = false;

}
;

SerialSwitch::~SerialSwitch() {
	// restore old port settings
	if (fd > 0)
		tcsetattr(fd, TCSANOW, &oldtio);
	close(fd);
}
;

void SerialSwitch::update(int state) {

//	ROS_INFO("enter update");
	if (state == 0) {
		unsigned char data[1] = { 0x61 };
//		unsigned int ret = 0;
		tcflush(fd, TCIFLUSH);
		write(fd, data, 1);
	}
	if (state == 1) {
		unsigned char data[1] = { 0x62 };
//		unsigned int ret = 0;
		tcflush(fd, TCIFLUSH);
		write(fd, data, 1);
	}
}

int SerialSwitch::getNumbeOfBalls() {

	unsigned char data[1] = { 'c' };
	unsigned int ret = 0;
	tcflush(fd, TCIFLUSH);
	write(fd, data, 1);

	unsigned char *buff_in;

	char c;

	if(mySelect(fd)){
		read(fd, &c, 1);
		return (int)c;
	}
//	std::cout << "getNumberOfBalls, ret = " << ret << " balls " << (int) c << std::endl;
}

void SerialSwitch::dump() {
	_dump = true;
}

bool SerialSwitch::isConnected() {
	if (connected) {
//		ROS_INFO("SERIAL CONNTECTED");
	} else {
//		ROS_INFO("SERIAL NOT CONNTECTED");
	}
	return connected;
}

bool SerialSwitch::mySelect(int file_desc) {

	int n;
	int max_fd;
	fd_set input;
	struct timeval timeout;

	/* Initialize the input set */
	FD_ZERO(&input);
	FD_SET(file_desc, &input);

	max_fd = file_desc + 1;

	timeout.tv_sec = 1;
	timeout.tv_usec = 0;

	/* Do the select */
	n = select(max_fd, &input, NULL, NULL, &timeout);

	/* See if there was an error */
	if (n < 0){
		perror("select failed");
		false;
	}
	else if (n == 0){
		puts("TIMEOUT");
		false;
	}
	else {
		/* We have input */
		if (FD_ISSET(fd, &input)){
			//  process_fd();
			return true;
		}
	}


}




