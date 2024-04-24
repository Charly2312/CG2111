#ifndef __CS_H
#define __CS_H

#include <Arduino.h>

class Led
{
private:
	byte _blue_led;	// PL3
	byte _red_led;	// PL4
	byte _green_led;	// PL5
public:	
	led() {};  // do not use

	Led(byte blue_led, byte red_led, byte green_led);
	
	void init();        // Turn on 

	void deactivate();  // off LED
	
	void shineWhite();

	void shineRed();

	void shineGreen();
};

#endif
