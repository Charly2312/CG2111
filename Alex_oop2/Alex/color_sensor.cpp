#include "color_sensor.h"

// constructor
Cs::Cs(SensorConfig& config, MP3* mp3)
  : _S0(config._S0), _S1(config._S1), _S2(config._S2), _S3(config._S3),
    _sensorOut(config._sensorOut), _redValue(0), _greenValue(0), _blueValue(0), 
    _mp3(mp3) {}

void Cs::setupColor() {
  // Set S0 - S3 as outputs
  DDRC |= 0b01111000;
  //pinMode(S0, OUTPUT);
  //pinMode(S1, OUTPUT);
  //pinMode(S2, OUTPUT);
  //pinMode(S3, OUTPUT);

  // Offing LED at the start
  led.deactivate(); 

  // Set Sensor output as input
  DDRG &= 0b11111110;
  //pinMode(sensorOut, INPUT);

  // Set Frequency scaling to 20%
  PORTC |= 0b00100000;
  PORTC &= 0b10111111;
  //digitalWrite(S0,HIGH);
  //digitalWrite(S1,LOW);

}

// Function to read Red Pulse Widths
unsigned long Cs::getRedPW() {
  // Set sensor to read Red only
  //digitalWrite(S2,LOW);
  //digitalWrite(S3,LOW);
  PORTC &= 0b11100111;
  // Define integer to represent Pulse Width
  unsigned long PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut, LOW);
  // Return the value
  return PW;
}

// Function to read Green Pulse Widths
unsigned long Cs::getGreenPW() {
  // Set sensor to read Green only
  //digitalWrite(S2,HIGH);
  //digitalWrite(S3,HIGH);
  PORTC |= 0b00011000;
  // Define integer to represent Pulse Width
  unsigned long PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut, LOW);
  // Return the value
  return PW;
}

// Function to read Blue Pulse Widths
unsigned long Cs::getBluePW() {
  // Set sensor to read Blue only
  //digitalWrite(S2,LOW);
  //digitalWrite(S3,HIGH);
  PORTC &= 0b11101111;
  PORTC |= 0b00001000;
  // Define integer to represent Pulse Width
  unsigned long PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut, LOW);
  // Return the value
  return PW;
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
  Led led(BLUELED, REDLED, GREENLED);

  led.init();

  //Serial.println(getDistance());
  if (rRatio < 0.4 && gRatio < 0.4 && bRatio < 0.4) {
    // white
    //Serial.println("white");
    color = 1;
    //shineWhite();
    led.shineWhite();
    _mp3.play_track(5);
    delay(3000);
  } else if (rRatio >= 0.39 && rRatio > gRatio && rRatio > bRatio) {  // 0.41, 0.30, 0.29 distance is 10 cm
    // red
    //Serial.println("red");
    color = 2;
    //shineRed();
    led.shineRed();
    _mp3.play_track(4);
    delay(4000);
  }

  else if (rRatio < 0.5 && gRatio > bRatio && gRatio > rRatio) {  //0.34, 0.34, 0.32 distance is 7 till 22 cm
    // green
    //Serial.println("green");
    color = 3;
    //shineGreen();
    led.shineGreen();
    _mp3.play_track(3);
    delay(4000);
  }
  delay(500);
  //DDRL &= 0b11000111;
  led.deactivate();
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
}