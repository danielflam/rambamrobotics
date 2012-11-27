

#include "Arduino.h"

#include <I2C.h>
#include <Servo.h>

#include <stopwatch.h>
#include <i2cperipheral.h>
#include <WiiCam.h>
#include <HMC5883.h>

#include "soccerboard.h"
#include "soccerMotors.h"
#include "soccerServo.h"

SoccerMotors motors;
SoccerServo<CAMERA_SERVOPIN> cameraServo;
HMC5883 compass;
Stopwatch compassStatusDelay(500);

// WiiCam Pixart camera
WiiCam wiiCam;
bool ballseen = false;
Stopwatch ballLastSeen(200);

int currentCameraY = 0;
int currentCameraX = 0;
byte previousSpinDirection;


void setup()
{
  // First set up these to prevent things from breaking/moving with no control.
  cameraServo.setup(100);
  
  motors.setup();
  previousSpinDirection = MOVDIR_SPINCLOCK;
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
 
   Serial.println("Initialization complete.");
      motors.setMotorPower(15);
      motors.moveBot(MOVDIR_SPINCLOCK);

}


int logcount = 0;
void loop()
{
  motors.update();

// Step 1. Align motors make sure they all rotate in the right direction
//  motors.testMotors();
//  return;
  
  //  testMotors();
  wiiCam.update();
  if (wiiCam.hasNewData())
  {
    logcount++;
    if (logcount > 50)
    {
      wiiCam.log();
      logcount = 0;
    }

//     int angle =  wiiCam.avgY>>5;
    if (wiiCam.avgY > 0)
    {
       if (ballseen == false)
       {
         ballLastSeen.reset();
         ballseen = true;
         // current angle is best guess
         currentCameraY = cameraServo.get();
       }
  
//       if (cameraAngleUpdate.timeout(STOPWATCH_RESTART ))
//       {
         int currentCameraAngleY = map(1023 - wiiCam.avgY, 0, 1023, 0, 10) - 5;    

         int currentCameraAngleX = map(wiiCam.avgX, 0, 1023, 0, 6) - 3;    

         currentCameraY = constrain( currentCameraY  + currentCameraAngleY , 60, 170);
         
//       }
         
       
       //currentCameraY = ( 7 * currentCameraY + map(1023-wiiCam.avgY, 1, 1022, 30, 170)) >> 3; // fast divide by 2^4 = 16 
       
       if (ballLastSeen.timeSinceReset() > 200)
       {
         if (currentCameraAngleX < -1)
         {
             motors.setMotorPower(55);

             motors.moveBot(MOVDIR_SPINANTI);
             previousSpinDirection = MOVDIR_SPINANTI;
         }
         else if (currentCameraAngleX < 0)
         {
             motors.setMotorPower(30);

             motors.moveBot(MOVDIR_FWD_AND_SPINANTI);
             previousSpinDirection = MOVDIR_SPINANTI;
         }
         else if (currentCameraAngleX > 1)
         {
             motors.setMotorPower(55);
             motors.moveBot(MOVDIR_SPINCLOCK);
             previousSpinDirection = MOVDIR_SPINCLOCK;
         }
         else if (currentCameraAngleX > 0)
         {
             motors.setMotorPower(30);
             motors.moveBot(MOVDIR_FWD_AND_SPINCLOCK);
             previousSpinDirection = MOVDIR_SPINCLOCK;
         }
         else
         {
            motors.setMotorPower(currentCameraY < 90 ? 60 : 130);
            motors.moveBot(MOVDIR_FWD);
         }

          cameraServo.set( currentCameraY );
       }
       else
       {
         motors.setMotorPower(35);
         motors.moveBot(previousSpinDirection);
       }
    }
    else
    {
        if (ballseen)
        {
           ballseen = false;
           motors.setMotorPower(35);
           motors.moveBot(previousSpinDirection);
        }
    }

     wiiCam.newDataProcessed();
  }  
  
  if (wiiCam.avgY == 0)
  {
      cameraServo.scan(85,155);
      motors.setMotorPower(35);
      motors.moveBot(previousSpinDirection);
  }
  
//  cameraServo.scan(65,140);


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

 
}

