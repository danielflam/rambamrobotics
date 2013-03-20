

#include "Arduino.h"

#include <I2C.h>
#include <Servo.h>

#include <stopwatch.h>
#include <i2cperipheral.h>
#include <WiiCam.h>
#include <HMC5883.h>
//#include <PID_v1.h>

#include <FlexiTimer2.h>
#include "soccerboard.h"
#include "soccerMotors.h"
#include "soccerServo.h"
#include "soccerPID.h"

#define MIN_MOTOR_POWER 0
#define MAX_MOTOR_POWER 255


enum EnumBallState {
  BALL_STATE_NOT_VISIBLE,
  BALL_STATE_GLIMPSE,
  BALL_STATE_VISIBLE,
  BALL_STATE_LOST
};  


char * ballStateName[4] = {
  "NOT_VISIBLE", "GLIMPSE", "VISIBLE", "LOST"};
EnumBallState ballstate;
EnumBallState newballstate; 
bool ballVisible;

PID pidy;
PID pidx;

SoccerMotors motors;
SoccerServo<CAMERA_SERVOPIN> cameraServo;
HMC5883 compass;
Stopwatch compassStatusDelay(500);
Stopwatch wiiStatusDelay(500);
Stopwatch logMotors(10);


// WiiCam Pixart camera
WiiCam wiiCam;
Stopwatch ballStateTime(200);


int currentCameraY = 0;
int currentCameraX = 0;
EnumMoveDir previousSpinDirection;
float lastSeenHeading = 0;
double motorPower;

float startupHeading;

void updateMotors()
{
  motors.update();
}

void getStartupHeading()
{
  int i = 0;
  Stopwatch compassStarup(1200);
  compassStarup.reset();
  while (!compassStarup.timeout())
  {
    compass.update();    
    if (compass.hasNewData())
    {
      if (i == 0)
      {
        startupHeading = compass.heading();
      }
      else
      {
        startupHeading = 0.5*(startupHeading + compass.heading());
      }
      compass.newDataProcessed();
    }
  }
}

void setup()
{
  ballstate = BALL_STATE_NOT_VISIBLE;
  newballstate = ballstate;
  ballStateTime.reset();

  // First set up these to prevent things from breaking/moving with no control.
  cameraServo.setup(130, 20, 130);

  // YYYYYYY
  pidy.setup(1.1, 0.4, 0.0005, DIRECT,0.0001);
  pidy.mySetpoint = 600.0; 
  pidy.SetOutputLimits(-1.0, 1.0);
  pidy.SetMode(AUTOMATIC);
  pidy.reset();

  // XXXXXXX
  pidx.setup(10, 2, 0.00002, DIRECT,0.01);
  pidx.mySetpoint = 412.0; 
  pidx.SetOutputLimits(-50.0, 50.0);
  pidx.SetMode(AUTOMATIC);
  pidx.reset();

  motorPower = 0.0;
  motors.setup();
  motors.setPowerMinMax(MIN_MOTOR_POWER, MAX_MOTOR_POWER);

  motors.setMotorPower(0);
  FlexiTimer2::set(1,0.001, updateMotors);
  FlexiTimer2::start();

  previousSpinDirection = MOVDIR_SPIN_CLOCKWISE;
  randomSeed(analogRead(0));

  Serial.begin(19200);
  delay(1000);
  Serial.println("starting");

  // Start up I2C:

  I2c.timeOut(100);
  I2c.pullup(0);
  I2c.setSpeed(1); // Fast I2c
  I2c.begin();


  wiiCam.setup();   
  compass.setup();


  getStartupHeading();

  motors.setMotorPower(15);
  motors.moveBot(MOVDIR_SPIN_CLOCKWISE);
  //motors.moveBot(MOVDIR_STOP);

  Serial.println("Initialization complete.");

}


void loop()
{
  //  motors.update();
  //  motors.setMotorPower(10);
  //  motors.moveBot( MOVDIR_FWD );
  //  delay(1000);
  //  motors.moveBot( MOVDIR_REV );
  //  delay(1000);
  //  return;

  compass.update();
  if (compass.hasNewData())
  {
    if (compassStatusDelay.timeout(STOPWATCH_RESTART))
    {
      compass.log();
      compassStatusDelay.reset();
    }    
    compass.newDataProcessed();
  }

  //  testMotors();
  wiiCam.update();
  if (wiiCam.hasNewData())
  {

    // LOGGING
    if (wiiStatusDelay.timeout(STOPWATCH_RESTART))    
    {

      // wiiCam.log();
      Serial.print("PID in: ");
      Serial.print(pidx.myInput);
      Serial.print(" out: ");
      Serial.print(pidx.myOutput);
      Serial.print(" motors: ");
      Serial.print(motors.getPower());

      Serial.print("\n");

      wiiStatusDelay.reset();
    }

    //calculate the call seen state

    ballVisible = wiiCam.avgY > 0;    

    if (ballstate == BALL_STATE_NOT_VISIBLE)
    {
      if (ballVisible)
      {
        newballstate = BALL_STATE_GLIMPSE;
      }
    }
    else if (ballstate == BALL_STATE_GLIMPSE)
    {
      if (!ballVisible)
      {
        newballstate = BALL_STATE_NOT_VISIBLE;
      }
      else 
      {
        if (ballStateTime.timeSinceReset() > 20)
        {
          newballstate = BALL_STATE_VISIBLE;
        }
      }
    }
    else if (ballstate == BALL_STATE_VISIBLE)
    {
      if (!ballVisible)
      {
        newballstate = BALL_STATE_LOST;
      }
    }
    else if (ballstate == BALL_STATE_LOST)
    {
      if (ballVisible)
      {
        newballstate = BALL_STATE_VISIBLE;
      }
      else 
      {
        if (ballStateTime.timeSinceReset() > 150)
        {
          newballstate = BALL_STATE_NOT_VISIBLE;
        }
      }
    }

    //      int currentCameraAngleY = map(1023 - wiiCam.avgY, 0, 1023, 0, 10) - 5;    
    //      int currentCameraAngleX = map(wiiCam.avgX, 0, 1023, 0, 6) - 3;    
    //      currentCameraY = constrain( currentCameraY  + currentCameraAngleY , 60, 170);
    //currentCameraY = ( 7 * currentCameraY + map(1023-wiiCam.avgY, 1, 1022, 30, 170)) >> 3; // fast divide by 2^4 = 16 
    //        if (PIDYOUT > 0)
    //          cameraServo.set( PIDYOUT );

    // stuff to be done once on every new camera frame goes here:
    //    if (newballstate == BALL_STATE_NOT_VISIBLE || newballstate == BALL_STATE_GLIMPSE)  .....
    //    else if (newballstate == BALL_STATE_VISIBLE || newballstate == BALL_STATE_LOST) ......


    // If we saw the ball then record the compass heading so we can go looking for it if
    // we are blind
    if (ballVisible)
    {
      pidy.myInput = int(wiiCam.avgY/10)*10.0 ; // quantize the input. 
      pidx.myInput = int(wiiCam.avgX/10)*10.0 ;
      lastSeenHeading = (lastSeenHeading + compass.heading())/2;
    }


    wiiCam.newDataProcessed();
  }

  // handle transitions
  // This is here because camera has lost ball and will not nitify again 
  if (ballstate == BALL_STATE_LOST && ballStateTime.timeSinceReset() > 150)
  {
    newballstate = BALL_STATE_NOT_VISIBLE;
  }

  if (ballstate != newballstate)
  {
    ballStateTime.reset();

    Serial.print("New ball state transition - ");
    Serial.print(ballStateName[ballstate]);
    Serial.print(" ==> ");
    Serial.print(ballStateName[newballstate]);
    Serial.println();

    if (newballstate == BALL_STATE_VISIBLE)
    {
      // pidy.reset();
    }

    ballstate = newballstate;

    // stuff to be done once on a transition goes here
    //    if (ballstate == BALL_STATE_NOT_VISIBLE || ballstate == BALL_STATE_GLIMPSE){}
    //    else if (ballstate == BALL_STATE_VISIBLE || ballstate == BALL_STATE_LOST){}
  }  



  if (ballstate == BALL_STATE_NOT_VISIBLE )
  {
    cameraServo.scan(70,135);

    motors.moveBot(  previousSpinDirection );
    motors.setMotorPower(50);

    //cameraServo.set(170);
  }
  if (ballstate == BALL_STATE_GLIMPSE)
  {
    float newheading = compass.heading();

    previousSpinDirection = compass.heading() > lastSeenHeading ?
    MOVDIR_SPIN_COUNTER_CLOCKWISE : MOVDIR_SPIN_CLOCKWISE;
    motors.setMotorPower(30);
    motors.moveBot(  previousSpinDirection );

  }
  else if (ballstate == BALL_STATE_VISIBLE )
  {
    pidy.Compute();

    cameraServo.add(pidy.myOutput);    


    pidx.Compute();
    //      motorPower = constrain(pidx.myOutput, -20,20);

    if ( abs(pidx.myInput - pidx.mySetpoint) < 100.0 && abs(pidx.myOutput) < 10)
    {
      if ( abs(startupHeading - compass.heading()) < 20)
        motors.setMotorPower(255);
      else
        motors.setMotorPower(60);
      motors.moveBot(  MOVDIR_FWD);
    }
    else if (pidx.myOutput > 1)
    {
      previousSpinDirection = MOVDIR_SPIN_CLOCKWISE;
      motors.setMotorPower(int(pidx.myOutput));
      motors.moveBot( MOVDIR_SPIN_CLOCKWISE  );
    }
    else if (pidx.myOutput < -1)
    {
      previousSpinDirection = MOVDIR_SPIN_COUNTER_CLOCKWISE;
      motors.setMotorPower(int(-pidx.myOutput));
      motors.moveBot( MOVDIR_SPIN_COUNTER_CLOCKWISE  );
    }
    else {
      // eliminate hunting
      motors.setMotorPower(0);
    }
  }
  else if (ballstate == BALL_STATE_LOST)
  {
    float newheading = compass.heading();

    previousSpinDirection = compass.heading() > lastSeenHeading ?
    MOVDIR_SPIN_COUNTER_CLOCKWISE : MOVDIR_SPIN_CLOCKWISE;
    motors.setMotorPower(30);
    motors.moveBot(  previousSpinDirection );
  }


}














