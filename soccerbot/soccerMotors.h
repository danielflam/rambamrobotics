#ifndef _SOCCER_MOTORS_H
#define _SOCCER_MOTORS_H

#define MOTOR_DIR_FWD 2
#define MOTOR_DIR_REV 1
#define MOTOR_DIR_STOP 0

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

#define MOTOR_1_A 5
#define MOTOR_1_B 4
#define MOTOR_2_A 3
#define MOTOR_2_B 2
#define MOTOR_3_A 7
#define MOTOR_3_B 6
#define MOTOR_4_A 8
#define MOTOR_4_B 9


class SoccerMotors
{
public:

  void setup()
  {
    //    timer::set(1, 1.0/2000, update);
    motorPower = 0;
    pinMode(MOTOR_1_A,OUTPUT);
    pinMode(MOTOR_1_B,OUTPUT);
    pinMode(MOTOR_2_A,OUTPUT);
    pinMode(MOTOR_2_B,OUTPUT);
    pinMode(MOTOR_3_A,OUTPUT);
    pinMode(MOTOR_3_B,OUTPUT);
    pinMode(MOTOR_4_A,OUTPUT);
    pinMode(MOTOR_4_B,OUTPUT);

    moveBot(MOVDIR_STOP);
    
    MinPower = 0;
    MaxPower = 255;
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
    Serial.print(yn);
    Serial.print(" ");
    Serial.print(qe);
    Serial.print(" ");
    
    Serial.println();
  }

  void update()
  {
    //    counter++;


    //
    // Use pulse density modulation to control the motor
    // see http://en.wikipedia.org/wiki/Pulse-density_modulation
    //
    yn = (motorPower >= qe) ? 255 : 0;
    qe = yn - motorPower + qe;
    liveMotorState = (yn == 255) ? motorState : MOVDIR_STOP;


    switch(liveMotorState)
    {
    case MOVDIR_STOP:
      setMotors(MOTOR_DIR_STOP,MOTOR_DIR_STOP,MOTOR_DIR_STOP,MOTOR_DIR_STOP);  
      break;

    case MOVDIR_FWD:    
      setMotors(MOTOR_DIR_FWD,MOTOR_DIR_FWD,MOTOR_DIR_REV,MOTOR_DIR_REV);
      break;

    case MOVDIR_REV:
      setMotors(MOTOR_DIR_REV,MOTOR_DIR_REV,MOTOR_DIR_FWD,MOTOR_DIR_FWD);
      break;

    case MOVDIR_RIGHT:     
      setMotors(MOTOR_DIR_REV,MOTOR_DIR_FWD,MOTOR_DIR_FWD,MOTOR_DIR_REV);
      break; 

    case MOVDIR_LEFT:     
      setMotors(MOTOR_DIR_FWD,MOTOR_DIR_REV,MOTOR_DIR_REV,MOTOR_DIR_FWD);
      break;

    case MOVDIR_SPIN_CLOCKWISE:     
      setMotors(MOTOR_DIR_REV,MOTOR_DIR_REV,MOTOR_DIR_REV,MOTOR_DIR_REV);
      break;

    case MOVDIR_SPIN_COUNTER_CLOCKWISE:     
      setMotors(MOTOR_DIR_FWD,MOTOR_DIR_FWD,MOTOR_DIR_FWD,MOTOR_DIR_FWD);
      break;

    case MOVDIR_FWD_AND_SPIN_CLOCK:    
      setMotors(MOTOR_DIR_FWD,MOTOR_DIR_REV,MOTOR_DIR_FWD,MOTOR_DIR_FWD);
      break;

    case MOVDIR_FWD_AND_SPIN_COUNTER_CLOCKWISE:    
      setMotors(MOTOR_DIR_FWD,MOTOR_DIR_REV,MOTOR_DIR_REV,MOTOR_DIR_REV);
      break;

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
    delay(2000); 
    // Use this to test the motors out of state machine
    digitalWrite(MOTOR_1_A,HIGH); 
    delay(1000); 
    digitalWrite(MOTOR_1_A,LOW);
    digitalWrite(MOTOR_1_B,HIGH); 
    delay(1000); 
    digitalWrite(MOTOR_1_B,LOW);
    
   
    digitalWrite(MOTOR_2_A,HIGH); 
    delay(1000); 
    digitalWrite(MOTOR_2_A,LOW);
    digitalWrite(MOTOR_2_B,HIGH); 
    delay(1000); 
    digitalWrite(MOTOR_2_B,LOW);
    digitalWrite(MOTOR_3_A,HIGH); 
    delay(1000); 
    digitalWrite(MOTOR_3_A,LOW);
    digitalWrite(MOTOR_3_B,HIGH); 
    delay(1000); 
    digitalWrite(MOTOR_3_B,LOW);
    digitalWrite(MOTOR_4_A,HIGH); 
    delay(1000); 
    digitalWrite(MOTOR_4_A,LOW);
    digitalWrite(MOTOR_4_B,HIGH); 
    delay(1000); 
    digitalWrite(MOTOR_4_B,LOW);
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

  void setMotors( byte motor_a_dir, byte motor_b_dir,  byte motor_c_dir, byte motor_d_dir)
  {
    setMotorPin( MOTOR_1_A, MOTOR_1_B, motor_a_dir);
    setMotorPin( MOTOR_2_A, MOTOR_2_B, motor_b_dir);
    setMotorPin( MOTOR_3_A, MOTOR_3_B, motor_c_dir);
    setMotorPin( MOTOR_4_A, MOTOR_4_B, motor_d_dir);
  }

  void setMotorPin( byte pinFwd, byte pinRev, byte motor_dir)
  {
    switch (motor_dir)
    {
    case MOTOR_DIR_STOP:
      digitalWrite(pinFwd, LOW);
      digitalWrite(pinRev, LOW);
      break;
    case MOTOR_DIR_REV:
      digitalWrite(pinFwd, LOW);
      digitalWrite(pinRev, HIGH);
      break;
    case MOTOR_DIR_FWD:
      digitalWrite(pinRev, LOW);
      digitalWrite(pinFwd, HIGH);
      break;
    }
  }

  int yn;
  int qe;

  EnumMoveDir motorState;
  EnumMoveDir liveMotorState;
  int motorPower;

  int MaxPower;
  int MinPower;

  unsigned long lastMotorStateUpdateTime;

};


#endif



