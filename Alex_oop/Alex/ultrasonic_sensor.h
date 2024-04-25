#ifndef __ULTRA_H
#define __ULTRA_H

#include <Arduino.h>

class Ultra
{
private:
	byte _echo;	// yellow // PD1 
	byte _trig;	// orange // PD0 
public:	
	Ultra() {};                   // do not use
	Ultra(byte echo, byte _trig);
  uint32_t getDistance();
};


#endif
