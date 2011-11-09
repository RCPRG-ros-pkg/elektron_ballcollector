/*
* serialswitch.cpp
*
* Created on: Oct 6, 2011
* Author: pmajcher
*/

#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <cstring>
#include <iostream>

#include "serialswitch.hpp"

using namespace std;



SerialSwitch::SerialSwitch(const std::string& port, int baud){
	connected = false;



	fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd >= 0) {
		tcgetattr(fd, &oldtio);

		// set up new settings
		struct termios newtio;
		memset(&newtio, 0, sizeof(newtio));
		newtio.c_cflag = CBAUD | CS8 | CLOCAL | CREAD | CSTOPB;
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

};

SerialSwitch::~SerialSwitch(){
	// restore old port settings
	if (fd > 0)
		tcsetattr(fd, TCSANOW, &oldtio);
	close(fd);
};

void SerialSwitch::update() {
	unsigned char data[1] = {0x40} ;

	unsigned int ret = 0;
	tcflush(fd, TCIFLUSH);
	write(fd, data, 1);

//	while (ret < sizeof(getdata))
//		ret += read(fd, ((char*) &getdata) + ret, sizeof(getdata) - ret);
}

void SerialSwitch::dump() {
	_dump = true;
	of.open("/tmp/odom_dump.txt");
	of << "aaaaaaa\n";
}

bool SerialSwitch::isConnected() {
	return connected;
}

