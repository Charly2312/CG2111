
#include <serialize.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>

#include "packet.h"
#include "constants.h"
#include "mp3.h"

MP3 mp3;

volatile TDirection dir;

// TCS230 or TCS3200 pins wiring to Arduino
#define S0 32 // PC5
#define S1 31 // PC6
#define S2 33 // PC4
#define S3 34 // PC3
#define sensorOut 41 // PG0

// LED pins
#define blueLED 46 // PL3
#define redLED 45 // PL4
#define greenLED 44 // PL5

#define ECHO 20 // yellow // PD1 // PA2
#define TRIG 21 // orange // PD0 // PA3

// Calibration Values - 3cm between sensor and color paper
// *Get these from Calibration Sketch
//distance is 10 - 11 cm
//blackPW = 890, 865, 680
//whitePW = 460, 485, 390
unsigned long redMin = 460; //190; // Red minimum value
unsigned long redMax = 890; //810; // Red maximum value
unsigned long greenMin = 485; //185; // Green minimum value
unsigned long greenMax = 865; //800; // Green maximum value
unsigned long blueMin = 390; //155; // Blue minimum value
unsigned long blueMax = 680; //644; // Blue maximum value

uint32_t color = 0;

//PI, for calculating circumference
#define PI 3.141592654

/*
 * Alex's configuration constants
 */

// Alex's lenght and breadth in cm
// These values are what we measured
#define ALEX_LENGTH 25.8
#define ALEX_BREADTH 16.0

//Alex's Diagonal. We compute and store this once since
// it is expensive to compute and never changes
float alexDiagonal = 30.35;

  //Alex's turning circumeference, calculated once.
  //Assume that Alex "turns on a dime"
  // PI * alexDiagonal
float alexCirc = PI * alexDiagonal;

// Number of ticks per revolution from the
// wheel encoder.
#define COUNTS_PER_REV 4

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC 22

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.

volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

// Left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

//Variables to keep track of whether we've moved
//a commanded distance
unsigned long deltaDist;
unsigned long newDist;

// Variables to keep track of our turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

// Variables for Color Pulse Width Measurements
volatile unsigned long redPW = 0;
volatile unsigned long greenPW = 0;
volatile unsigned long bluePW = 0;

// Variables for final Color values
volatile unsigned long redValue;
volatile unsigned long greenValue;
volatile unsigned long blueValue;

float distance = 0.0;

/*
 * 
 * Alex Communication Routines.
 * 
 */

TResult readPacket(TPacket *packet) {
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);
}

void sendStatus() {
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;

  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;

  colorSense();
  statusPacket.params[10] = color;
  statusPacket.params[11] = redValue;
  statusPacket.params[12] = greenValue;
  statusPacket.params[13] = blueValue;

  statusPacket.params[14] = getDistance();
  sendResponse(&statusPacket);
}

void sendDist() {
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_DISTANCE;

  statusPacket.params[14] = getDistance();
  sendResponse(&statusPacket);
}

void sendMessage(const char *message) {
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

//from week 9 studio 1
//only use dbprintf for debug printing
//DO NOT USE SERIAL.WRITE
void dbprintf(char *format, ...) {
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket() {
  // Tell the Pi that it sent us a packet with a bad
  // magic number.

  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
}

void sendBadChecksum() {
  // Tell the Pi that it sent us a packet with a bad
  // checksum.

  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand() {
  // Tell the Pi that we don't understand its
  // command sent to us.

  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse() {
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK() {
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet) {
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

/*void sendColor() { 
  TPacket statusPacket;
  status.packetType = PACKET_TYPE_MESSAGE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  statusPacket.params[10] = color;
  sendResponse(&statusPacket); 
} */

/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 18 and 19
void enablePullups() {
  // Use bare-metal to enable the pull-up resistors on pins
  // 19 and 18. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.
  DDRD &= 0b11110011;   //setting PD2 and PD3 as inputs
  PORTD |= 0b00001100;  //drive PD2 and PD3 HIGH
}

// Functions to be called by INT2 and INT3 ISRs.
ISR(INT3_vect) {  //leftISR
  switch (dir) {
      //Alex is moving forward
    //increment leftForwardTicks and calculate forwardDist
    case FORWARD:
      leftForwardTicks++;
      forwardDist = (unsigned long)((float)leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
      break;

    //Alex is moving backward
    //increment leftReverseTicks and calculate reverseDist
    case BACKWARD:
      leftReverseTicks++;
      reverseDist = (unsigned long)((float)leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
      break;

    //Alex is moving to the left
    //increment leftForwardTicksTurns
    case LEFT:
      leftReverseTicksTurns++;
      break;

    //Alex is moving to the right
    //increment RightReverseTicksTurns
    case RIGHT:
      leftForwardTicksTurns++;
      break;
  }
}

ISR(INT2_vect) {  //rightISR
  switch (dir) {
      //Alex is moving forward
    //increment rightForwardTicks
    case FORWARD:
      rightForwardTicks++;
      break;

    //Alex is moving backward
    //increment rightReverseTicks
    case BACKWARD:
      rightReverseTicks++;
      break;

    //Alex is moving to the left
    //increment rightReverseTicksTurns
    case LEFT:
      rightForwardTicksTurns++;
      break;

    //Alex is moving to the right
    //increment rightForwardTicksTurns
    case RIGHT:
      rightReverseTicksTurns++;
      break;
  }
}

void setupColor() {
  // Set S0 - S3 as outputs
	DDRC |= 0b01111000;
  //pinMode(S0, OUTPUT);
	//pinMode(S1, OUTPUT);
	//pinMode(S2, OUTPUT);
	//pinMode(S3, OUTPUT);

  // Offing LED
  DDRL &= 0b11000111;

	// Set Sensor output as input
  DDRG &= 0b11111110;
	//pinMode(sensorOut, INPUT);

	// Set Frequency scaling to 20%
  PORTC |= 0b00100000;
  PORTC &= 0b10111111;
	//digitalWrite(S0,HIGH);
	//digitalWrite(S1,LOW);

  DDRD |= 0b00000001;
  DDRD &= 0b11111101;	
	// Setup Serial Monitor
}

// Set up the external interrupt pins INT2 and INT3
// for falling edge triggered. Use bare-metal.
void setupEINT() {
  // Use bare-metal to configure pins 18 and 19 to be
  // falling edge triggered. Remember to enable
  // the INT2 and INT3 interrupts.
  // Hint: Check pages 110 and 111 in the ATmega2560 Datasheet.
  cli();
  EICRA |= 0b10100000;  //set INT2 and INT3 to falling edge
  EIMSK |= 0b00001100;  //enable INT2 and INT3
  sei();
}

// Implement the external interrupt ISRs below.
// INT3 ISR should call leftISR while INT2 ISR
// should call rightISR.


// Implement INT2 and INT3 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial() {
  // To replace later with bare-metal.
  Serial.begin(9600);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using the other UARTs
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial() {
  // Empty for now. To be replaced with bare-metal code
  // later on.
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char *buffer) {

  int count = 0;

  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs

  while (Serial.available())
  buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len) {
  Serial.write(buffer, len);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters() {
  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;
  leftForwardTicksTurns = 0;
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightReverseTicksTurns = 0;
  leftRevs = 0;
  rightRevs = 0;
  forwardDist = 0;
  reverseDist = 0;
}

// Clears one particular counter
void clearOneCounter(unsigned long param) {
  param = 0;
}
// Intialize Alex's internal states

void initializeState() {
  clearCounters();
}

void handleCommand(TPacket *command) {
  switch (command->command) {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      forward((double)command->params[0], (float)command->params[1]);
      break;
    case COMMAND_REVERSE:
      sendOK();
      backward((double)command->params[0], (float)command->params[1]);
      break;
    case COMMAND_TURN_LEFT:
      sendOK();
      left((double)command->params[0], (float)command->params[1]);
      break;
    case COMMAND_TURN_RIGHT:
      sendOK();
      right((double)command->params[0], (float)command->params[1]);
      break;
    case COMMAND_STOP:
      sendOK();
      stop();
      break;
    case COMMAND_GET_STATS:
      sendOK();
      sendStatus();
      break;
    case COMMAND_CLEAR_STATS:
      sendOK();
      clearOneCounter(command->params[0]);
      break;
    case COMMAND_GET_DISTANCE:
      sendOK();
      sendDist();

    default:
      sendBadCommand();
  }
}

void waitForHello() {
  int exit = 0;

  while (!exit) {
    TPacket hello;
    TResult result;

    do {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK) {
      if (hello.packetType == PACKET_TYPE_HELLO) {
        sendOK();
        exit = 1;
      } else
        sendBadResponse();
    } else if (result == PACKET_BAD) {
      sendBadPacket();
    } else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  }  // !exit
}

void setup() {
  // put your setup code here, to run once:
	alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  cli();
  setupColor();
  setupEINT();
  setupSerial();
  startSerial();
  enablePullups();
  initializeState();
  sei();
  mp3.begin();
  mp3.volume(30);
}

void handlePacket(TPacket *packet) {
  switch (packet->packetType) {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

// Function to read Red Pulse Widths
unsigned long getRedPW() {
	// Set sensor to read Red only
  PORTC &= 0b11100111;
	//digitalWrite(S2,LOW);
	//digitalWrite(S3,LOW);
	// Define integer to represent Pulse Width
	unsigned long PW;
	// Read the output Pulse Width
	PW = pulseIn(sensorOut, LOW);
	// Return the value
	return PW;
}

// Function to read Green Pulse Widths
unsigned long getGreenPW() {
	// Set sensor to read Green only
  PORTC |= 0b00011000;
	//digitalWrite(S2,HIGH);
	//digitalWrite(S3,HIGH);
	// Define integer to represent Pulse Width
	unsigned long PW;
	// Read the output Pulse Width
	PW = pulseIn(sensorOut, LOW);
	// Return the value
	return PW;
}

// Function to read Blue Pulse Widths
unsigned long getBluePW() {
	// Set sensor to read Blue only
	PORTC &= 0b11101111;
  PORTC |= 0b00001000;
  //digitalWrite(S2,LOW);
	//digitalWrite(S3,HIGH);
	// Define integer to represent Pulse Width
	unsigned long PW;
	// Read the output Pulse Width
	PW = pulseIn(sensorOut, LOW);
	// Return the value
	return PW;
}

void shineWhite() {
  //change 245 to 0 for all
  analogWrite(redLED, 0); 
  analogWrite(greenLED, 0); 
  analogWrite(blueLED, 0);  
}

void shineRed() {
  analogWrite(redLED, 50);
  analogWrite(greenLED, 255);
  analogWrite(blueLED, 255);
}

void shineGreen() {
  analogWrite(redLED, 200);
  analogWrite(greenLED, 0);
  analogWrite(blueLED, 255);
}

void offLED() {
  analogWrite(redLED, 0);
  analogWrite(greenLED, 0);
  analogWrite(blueLED, 0);
}

void checkColor(unsigned long red, unsigned long green, unsigned long blue) {
  float sum = (float)(red+green+blue);
  float rRatio = fabs(red/sum);
  float gRatio = fabs(green/sum);
  float bRatio = fabs(blue/sum);
  /*Serial.print("RedRatio = ");
	Serial.print(rRatio);
	Serial.print(" - GreenRatio = ");
	Serial.print(gRatio);
	Serial.print(" - BlueRatio = ");
	Serial.println(bRatio);*/
  DDRL |= 0b00111000;

  //Serial.println(getDistance());
  if (rRatio < 0.4 && gRatio < 0.4 && bRatio < 0.4) { 
    // white
    //Serial.println("white");
    color = 1;
    shineWhite();
    mp3.play_track(5);
    delay(3000);
  }
  else if (rRatio >= 0.39 && rRatio > gRatio && rRatio > bRatio) { // 0.41, 0.30, 0.29 distance is 10 cm 
    // red
    //Serial.println("red");
    color = 2;
    shineRed();
    mp3.play_track(4);
    delay(4000);
  }
  
  else if (rRatio < 0.5 && gRatio > bRatio && gRatio > rRatio) { //0.34, 0.34, 0.32 distance is 7 till 22 cm
    // green
    //Serial.println("green");
    color = 3;
    shineGreen();
    mp3.play_track(3);
    delay(4000);
  }
  //else if ((rRatio >= 0.4 && gRatio > 0.20 && bRatio > 0.35) || (gRatio < 0.3 && bRatio > 0.3)) {
    // purple
  //  Serial.println("purple");}
  //else if (rRatio < 0.4 && bRatio >= 0.35) {
    // blue
  //  Serial.println("blue");}
  //else if (rRatio > 0.45 && gRatio < 0.3) {
    // orange
  //  Serial.println("orange");} 
  delay(500);
  DDRL &= 0b11000111;
}

uint32_t getDistance() {
  PORTD |= 0b00000001;
  //digitalWrite(TRIG, HIGH);
  delay(100);
  PORTD &= 0b11111110;
  //digitalWrite(TRIG, LOW);
  float microsecs = pulseIn(ECHO, HIGH);
  uint32_t cms = (uint32_t)(microsecs * 0.0345 / 2);
  return cms; 
}

void colorSense() { //max distance is 13cm
  // Read Red value
	redPW = getRedPW();
	// Map to value from 0-255
	redValue = abs(map(redPW, redMin,redMax,255,0));
	// Delay to stabilize sensor
	delay(200);

	// Read Green value
	greenPW = getGreenPW();
	// Map to value from 0-255
	greenValue = abs(map(greenPW, greenMin,greenMax,255,0));
	// Delay to stabilize sensor
	delay(200);

	// Read Blue value
	bluePW = getBluePW();
	// Map to value from 0-255
	blueValue = abs(map(bluePW, blueMin,blueMax,255,0));
	// Delay to stabilize sensor
	delay(200);

  /*Serial.print("RedPW = ");
	Serial.print(redPW);
	Serial.print(" - GreenPW = ");
	Serial.print(greenPW);
	Serial.print(" - BluePW = ");
	Serial.println(bluePW);*/
  
  checkColor(redValue, greenValue, blueValue); 
}

void loop() {
  // Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2
  //forward(0, 100);

  // Uncomment the code below for Week 9 Studio 2
  
  
  // put your main code here, to run repeatedly:
  TPacket recvPacket;  // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);

  if (result == PACKET_OK) {
    handlePacket(&recvPacket);
  } else if (result == PACKET_BAD) {
    sendBadPacket();
  } else if (result == PACKET_CHECKSUM_BAD) {
    sendBadChecksum();
  }

  //error occurs when we call the function
  if (deltaDist > 0) {
    if (dir == FORWARD) {
      if (forwardDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    } else if (dir == BACKWARD) {
      if (reverseDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    } else if (dir == STOP) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }

  if (deltaTicks > 0) {
    if (dir == LEFT) {
      if (leftReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    } else if (dir == RIGHT) {
      if (rightReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    } else if (dir == STOP) {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }
  //left(50,50);
  //colorSense();
}
