#include "ultrasonic_sensor.h"

Ultra::Ultra(byte echo, byte _trig) 
{
  _echo = echo;
  _trig = trig;
  // Set trig pin as an output 
  DDRD |= 0b00000001;
  // Set echo pin as an input 
  DDRD &= 0b11111101;
}

uint32_t Ultra::getDistance() {
  //digitalWrite(TRIG, HIGH);
  PORTD |= 0b00000001;
  delay(100);
  //digitalWrite(TRIG, LOW);
  PORTD &= 0b11111110;
  // replace with oop
  float microsecs = pulseIn(_echo, HIGH);
  uint32_t cms = (uint32_t)(microsecs * 0.0345 / 2);
  return cms; 
}