#ifndef _SOCCER_MOTORS_H
#define _SOCCER_MOTORS_H

#define MOTOR_DIR_FWD 2
#define MOTOR_DIR_REV 1
#define MOTOR_DIR_STOP 0

#include <AFMotor/AFMotor.h>

enum EnumMoveDir 
{ 
  MOVDIR_STOP,
  MOVDIR_FWD,
  MOVDIR_REV,
  MOVDIR_RIGHT,
  MOVDIR_LEFT,
  MOVDIR_SPIN_CLOCKWISE,
  MOVDIR_SPIN_COUNTER_CLOCKWISE,
  MOVDIR_FWD_AND_SPIN_CLOCK ,
  MOVDIR_FWD_AND_SPIN_COUNTER_CLOCKWISE,
  MOVDIR_COUNT
};


class SoccerMotors
{
public:
  SoccerMotors() :
	  motor1(1, MOTOR12_1KHZ),
	  motor2(2, MOTOR12_1KHZ),
	  motor3(3, MOTOR12_1KHZ),
	  motor4(4, MOTOR12_1KHZ)
{
	motorPower = 0;
	MinPower = 0;
	MaxPower = 255;

	motors[0] = &motor1;
	motors[1] = &motor2;
	motors[2] = &motor3;
	motors[3] = &motor4;
  }

  void setup()
  {

    motorPower = 0;

    
    MinPower = 0;
    MaxPower = 255;
    moveBot(MOVDIR_STOP);

	for (int i=0;i<4;i++)
	{
	//	motors[i]->setup();
	}
  }

  void setPowerMinMax(int minpower, int maxpower)
  {
    MinPower = minpower;
    MaxPower = maxpower;
    motorPower = constrain(motorPower, MinPower, MaxPower);

  }
  
  void log()
  {
    Serial.print(motorPower);
    Serial.print(" ");
    
    Serial.println();
  }

  void rawDrive(int M1, int M2, int M3, int M4)
  {

//	  motor1.set(M1);
//	  motor2.set(M2);
//	  motor3.set(M3);
//	  motor4.set(M4);
  }

  void directionalDrive(double Heading, double Power, double Rotation)
  {
	double X, Y;
	
	X = Power*cos(PI*Heading/180.0);
	Y = Power*sin(PI*Heading/180.0);
	holonomicDrive(X, Y, Rotation, Power);
  }

  void holonomicDrive(int X, int Y, int D, int P)
  {
	  long m[4];
	  int i = 0;

		float maxpower;

		P = constrain(P, 0, 255);

		m[0] = X - Y - D;
		m[1] = X + Y - D;    
		m[2] = -X + Y - D;    
		m[3] = -X - Y - D;    

		maxpower = 0;
		for (i=0; i<4; i++)
		{
		  if (maxpower < abs(m[i]))
			maxpower = abs(m[i]);
		}

		if (maxpower > P)
		{
		  for (i=0; i<4; i++)
		  {
			  m[i] = map(m[i], -maxpower,maxpower, -P,P); 
		  }
		}

	  for (i = 0; i < 4; i++)
	  {

		if (abs(m[i]) <= 1.0)
		{
			motors[i]->run(RELEASE);
			motors[i]->setSpeed(0);
		}
		else if (m[i] > 0)
		{
			motors[i]->run(FORWARD);
			motors[i]->setSpeed(int(m[i]));
		}
		else
		{
			motors[i]->run(BACKWARD);
			motors[i]->setSpeed(-int(m[i]));
		}
	  }
  }


  void moveBot( EnumMoveDir move_dir)
  {
    motorState = move_dir;
    //    liveMotorState = move_dir;
  }

  void setMotorPower(int power)
  {
    motorPower = constrain(power, MinPower, MaxPower);

  }

  void addMotorPower(int power)
  {
    motorPower = constrain(motorPower + power, MinPower, MaxPower);
  }
  
  int getPower()
  {
    return motorPower;
  }

  void testMotors()
  {
    //delay(2000);
    // Use this to test the motors out of state machine

    for (int i = 0; i<4; i++)
    {
    motors[i]->setSpeed(255);
	 motors[i]->run(FORWARD); // turn it on going forward
	 delay(1000);
	 motors[i]->run(BACKWARD); // the other way
	 delay(1000);
	 motors[i]->run(RELEASE); // stopped
	 delay(1000);
	 motors[i]->setSpeed(0);

    }
  }


  void testAllStates()
  {
    if ( (millis()-lastMotorStateUpdateTime) > 1000 )
    {

      // test all motor states one by one
      motorState = EnumMoveDir(int(motorState) + 1);
      if (motorState == MOVDIR_COUNT)
      {
        motorState = MOVDIR_STOP;
      }
      moveBot(motorState);

      lastMotorStateUpdateTime = millis();
    }
  }  

  void moveRandom()
  {
    if ( (millis()-lastMotorStateUpdateTime) > 1000 )
    {
      // move to random states
      moveBot(EnumMoveDir(random(1,6)));
      lastMotorStateUpdateTime = millis();
    }
  }


private:


  EnumMoveDir motorState;
  EnumMoveDir liveMotorState;
  int motorPower;

  int MaxPower;
  int MinPower;

  unsigned long lastMotorStateUpdateTime;
	
  AF_DCMotor motor1;
  AF_DCMotor motor2;
  AF_DCMotor motor3;
  AF_DCMotor motor4;

  AF_DCMotor * motors[4];
};


#endif



