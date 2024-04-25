#include "led.h"

Led::Led(byte blue_led, byte red_led, byte green_led) {
  _blue_led = _blue_led;
  _red_led = red_led;
  _green_led = green_led;

  TCNT0 = 0;
  TIMSK0 |= 
}

void Led::init() {
  DDRL |= 0b00111000;
}

void Led::deactivate() {
  DDRL &= 0b11000111;
}

void Led::shineWhite() {
  //change 245 to 0 for all
  analogWrite(_blue_led, 0);
  analogWrite(_red_led, 0);
  analogWrite(_green_led, 0);
}

void Led::shineRed() {
  analogWrite(_blue_led, 255);
  analogWrite(_red_led, 50);
  analogWrite(_green_led, 255);
}

void Led::shineGreen() {
  analogWrite(_blue_led, 255);
  analogWrite(_red_led, 200);
  analogWrite(_green_led, 0);
}