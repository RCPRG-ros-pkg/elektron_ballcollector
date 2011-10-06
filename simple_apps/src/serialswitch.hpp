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
#define BAUD B115200
 
// port
#define PORT ="/dev/ttyUSB0"
 
struct tgetstate{
     
     int16_t state;
}

struct tsetstate{
    int16_t state;
}

class SerialSwitch{

public:
 //   SerialSwitch(const std::string& port, int baud = BAUD);
    SerialSwitch();
    ~SerialSwitch();
    
 //  void setState(int state);
 //   void getState(int &state);
    
 //   bool isConnected();
    
private:
    // serial port descriptior
    int fd;
    bool connected;
    
    tsetstate setstate;
    tgetstate getstate;
    
    
    bool _dump;
}

#endif /*	ELEKTRON_HPP_	*/











