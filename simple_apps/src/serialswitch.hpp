/*
 * serialswitch.hpp
 *
 *  Created on: Oct 6, 2011
 *	Author pmajcher
 *
 */
 
#ifndef SERIALSWITCH_HPP_
#define SERIALSWITCH_HPP_
 
 
#include <stdint.h>
#include <termios.h>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <fstream>

 
// baudrate
#define BAUD B19200
 
// port
#define PORT ="/dev/ttyUSB0"
 
struct tgetstate{
     
     int16_t state;
};

struct tsetstate{
    int16_t state;
};

class SerialSwitch{

public:
    	SerialSwitch(const std::string& port, int baud = BAUD);

	~SerialSwitch();
	void update();
   
    
   	bool isConnected();
    	void dump();
private:

// serial port descriptior
	int fd;
	struct termios oldtio;
	bool connected;
   	bool _dump;
};

#endif /*	ELEKTRON_HPP_	*/











