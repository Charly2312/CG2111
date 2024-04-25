#include "color_sensor.h"

// constructor
Cs::Cs(SensorConfig& config, MP3* mp3)
  : _S0(config._S0), _S1(config._S1), _S2(config._S2), _S3(config._S3),
    _sensorOut(config._sensorOut),
     _redValue(0), _greenValue(0), _blueValue(0),
    _mp3(mp3),
    _color(0)
{
  DDRC |= 0b01111000;    // Set S0 - S3 as outputs
  DDRG &= 0b11111110;    // Set Sensor output as input

  DDRL &= 0b11000111;    // set three LED pins as inputs to disable them

  // Set Frequency scaling to 20%(), S0->HIGH S1->LOW
  PORTC |= 0b00100000;
  PORTC &= 0b10111111;
}


// Function to read Red Pulse Widths
unsigned long Cs::getRedPW() {
  // Setting RED filtered photodiodes for reading only red
  PORTC &= 0b11100111;    // S2,S3(LOW, LOW)
  unsigned long red_pw;    

  // Read the output Pulse Width
  red_pw = pulseIn(sensorOut, LOW);
  return red_pw;
}

// Function to read Green Pulse Widths
unsigned long Cs::getGreenPW() {
  // Setting GREEN filtered photodiodes for reading only green
  PORTC |= 0b00011000;    // S2,S3(LOW, HIGH)

  unsigned long green_pw;      
  // Read the output Pulse Width
  green_pw = pulseIn(sensorOut, LOW);
  return green_pw;
}

// Function to read Blue Pulse Widths
unsigned long Cs::getBluePW() {
  // Setting BLUE filtered photodiodes for reading only blue. S2,S3(LOW, HIGH)
  PORTC &= 0b11101111;
  PORTC |= 0b00001000;

  unsigned long blue_pw;
  // Read the output Pulse Width
  blue_pw = pulseIn(sensorOut, LOW);
  return blue_pw;
}

void Cs::checkColor() {
  float sum = (float)(_redValue + _greenValue + _blueValue);
  float rRatio = fabs(_redValue / sum);
  float gRatio = fabs(_greenValue / sum);
  float bRatio = fabs(_blueValue / sum);
  /*Serial.print("RedRatio = ");
  Serial.print(rRatio);
  Serial.print(" - GreenRatio = ");
  Serial.print(gRatio);
  Serial.print(" - BlueRatio = ");
  Serial.println(bRatio);*/
  //DDRL |= 0b00111000;

  // From led.h
  Led led(BLUELED, REDLED, GREENLED);   // declare a class led

  led.init();   // Enable led

  //Serial.println(getDistance());
  if (rRatio < 0.4 && gRatio < 0.4 && bRatio < 0.4) {
    // white
    //Serial.println("white");
    _color = 1;
    //shineWhite();
    led.shineWhite();
    _mp3.play_track(5);   
    delay(3000);    // delay 3s when led will shine white and wait for track to be played
  } else if (rRatio >= 0.39 && rRatio > gRatio && rRatio > bRatio) {  // 0.41, 0.30, 0.29 distance is 10 cm
    // red
    //Serial.println("red");
    _color = 2;
    //shineRed();
    led.shineRed();
    _mp3.play_track(4);
    delay(4000);      // delay 4s when led will shine red and wait for track to be played
  }

  else if (rRatio < 0.5 && gRatio > bRatio && gRatio > rRatio) {  //0.34, 0.34, 0.32 distance is 7 till 22 cm
    // green
    //Serial.println("green");
    _color = 3;
    //shineGreen();
    led.shineGreen();
    _mp3.play_track(3); // delay 3s when led will shine green and wait for track to be played
    delay(4000);
  }
  delay(500);   
  //DDRL &= 0b11000111;
  led.deactivate();   // Disable led 
}


void Cs::colorSense() {
  // max distance is 13cm
  // Read Red value
  unsigned long redPW = getRedPW();
  // Map to value from 0-255
  redValue = abs(map(redPW, REDMIN, REDMAX, 255, 0));
  // Delay to stabilize sensor
  delay(200);

  // Read Green value
  unsigned long greenPW = getGreenPW();
  // Map to value from 0-255
  greenValue = abs(map(greenPW, GREENMIN, GREENMAX, 255, 0));
  // Delay to stabilize sensor
  delay(200);

  // Read Blue value
  unsigned long bluePW = getBluePW();
  // Map to value from 0-255
  blueValue = abs(map(bluePW, BLUEMIN, BLUEMAX, 255, 0));
  // Delay to stabilize sensor
  delay(200);

  /*Serial.print("Red = ");
  Serial.print(redValue);
  Serial.print(" - Green = ");
  Serial.print(greenValue);
  Serial.print(" - Blue = ");
  Serial.println(blueValue);*/

  checkColor();
}

unsigned long Cs::get_redValue() {
  return _redValue;
}

unsigned long Cs::get_blueValue() {
  return _blueValue;
}

unsigned long Cs::get_greenValue() {
  return _greenValue;

uint32_t get_color() {
  return _color;
}
}