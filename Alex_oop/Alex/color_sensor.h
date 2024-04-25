#ifndef __CS_H
#define __CS_H

#include <Arduino.h>
#include "led.h"
#include "mp3.h"

// LED pins
#define BLUELED 46   // PL3
#define REDLED 45    // PL4
#define GREENLED 44  // PL5

// *Get these from Calibration Sketch
//distance is 10 - 13 cm
//blackPW = 890, 865, 680
//whitePW = 460, 485, 390
#define REDMIN 460;    // Red minimum value
#define REDMAX 890;    // Red maximum value
#define GREENMIN 485;  // Green minimum value
#define GREENMAX 865;  // Green maximum value
#define BLUEMIN 390;   // Blue minimum value
#define BLUEMAX 680;   // Blue maximum value

class MP3;  // declare a class MP3

// define SensorConfig struct
struct SensorConfig {
  byte _S0, _S1, _S2, _S3, _sensorOut;
};

class Cs {
private:
  byte _S0, _S1, _S2, _S3, _sensorOut;  // GPIO 
  unsigned long _redValue, _greenValue, _blueValue;  // 
  uint32_t _color;  // Indication color: 1-> white, 2 -> red, 3-> green
  MP3* _mp3; // store the pointer from MP3 passed from Alex.ino

public:
  Cs(){};  // do not use

  Cs(SensorConfig& config, MP3* mp3);  // constructor

  void setupColor();

  void checkColor();

  void colorSense();

  unsigned long get_redValue();

  unsigned long get_blueValue();

  unsigned long get_greenValue();

  uint32_t get_color();
};


#endif
