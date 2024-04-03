#include <AFMotor.h>

// Motor control
#define FRONT_LEFT   4 // M4 on the driver shield
#define FRONT_RIGHT  1 // M1 on the driver shield
#define BACK_LEFT    3 // M3 on the driver shield
#define BACK_RIGHT   2 // M2 on the driver shield

AF_DCMotor motorFL(FRONT_LEFT);
AF_DCMotor motorFR(FRONT_RIGHT);
AF_DCMotor motorBL(BACK_LEFT);
AF_DCMotor motorBR(BACK_RIGHT);

void move(float speed, int direction)
{
  int speed_scaled = (speed/100.0) * 255;
  motorFL.setSpeed(speed_scaled);
  motorFR.setSpeed(speed_scaled);
  motorBL.setSpeed(speed_scaled);
  motorBR.setSpeed(speed_scaled);

  switch(direction)
    {
      case BACK:
        motorFL.run(BACKWARD);
        motorFR.run(BACKWARD);
        motorBL.run(FORWARD);
        motorBR.run(FORWARD); 
      break;
      case GO:
        motorFL.run(FORWARD);
        motorFR.run(FORWARD);
        motorBL.run(BACKWARD);
        motorBR.run(BACKWARD); 
      break;
      case CW: //turn right
        motorFL.run(FORWARD);
        motorFR.run(BACKWARD);
        motorBL.run(BACKWARD);
        motorBR.run(FORWARD); 
      break;
      case CCW: //turn left
        motorFL.run(BACKWARD);
        motorFR.run(FORWARD);
        motorBL.run(FORWARD);
        motorBR.run(BACKWARD); 
      break;
      case STOP:
      default:
        motorFL.run(STOP);
        motorFR.run(STOP);
        motorBL.run(STOP);
        motorBR.run(STOP); 
    }
}

//New function to estimate number of wheel ticks
// needed to turn an angle
unsigned long computeDeltaTicks(float ang) {
  // We will assume that angular distance moved = linear distance moved in one wheel
  // revolution. This is probably incorrect but simplifies caluclation.
  // # of wheel revs to make on full 360 turn is vincentCirc / WHEEL_CIRC
  // This is for 360 degrees. For ang degrees it will be (ang * alexCirc) / (360 * WHEEL_CIRC)
  // To convert to ticks, we multiply by COUNTS_PER_REV.

  unsigned long ticks = (unsigned long)((ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC)) ;
  return ticks;
}

void forward(float dist, float speed)
{
  if(dist > 0) {
 	 	deltaDist = dist;
  } else 
 	 	deltaDist=9999999; 
  }
 	newDist = forwardDist + deltaDist; 

  dir = (TDirection) FORWARD;
  move(speed, FORWARD);
}

//reverse
void backward(float dist, float speed)
{
  if(dist > 0) {
 	 	deltaDist = dist;  	
  } else 
 	 	deltaDist=9999999; 
  }
 	newDist = reverseDist + deltaDist; 

  dir = (TDirection) BACKWARD;
  move(speed, BACKWARD);
}

//turn left, ccw
void left(float ang, float speed) {
  if (ang == 0) {
    deltaTicks = 99999999;
  } else
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = leftReverseTicksTurns + deltaTicks;

  dir = (TDirection) LEFT;
  move(speed, CCW);
}

void right(float ang, float speed) {
  if (ang == 0) {
    deltaTicks = 99999999;
  } else {
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = rightReverseTicksTurns + deltaTicks;

  dir = (TDirection) RIGHT;
  move(speed, CW);
}

void stop()
{
  dir = (TDirection) STOP;
  move(0, STOP);
}

