#ifndef _SOCCER_MOTORS_H
#define _SOCCER_MOTORS_H

#define MOTOR_DIR_FWD 2
#define MOTOR_DIR_REV 1
#define MOTOR_DIR_STOP 0

#define MOVDIR_STOP 0
#define MOVDIR_FWD 1
#define MOVDIR_REV 2
#define MOVDIR_RIGHT 3
#define MOVDIR_LEFT 4
#define MOVDIR_SPINCLOCK 5
#define MOVDIR_SPINANTI 6
#define MOVDIR_FWD_AND_SPINCLOCK 7
#define MOVDIR_FWD_AND_SPINANTI 8

#define MOTOR_1_A 2
#define MOTOR_1_B 3
#define MOTOR_2_A 4
#define MOTOR_2_B 5
#define MOTOR_3_A 6
#define MOTOR_3_B 7
#define MOTOR_4_A 8
#define MOTOR_4_B 9


class SoccerMotors
{
public:

  void setup()
  {
      motorPower = 5;
      pinMode(MOTOR_1_A,OUTPUT);
      pinMode(MOTOR_1_B,OUTPUT);
      pinMode(MOTOR_2_A,OUTPUT);
      pinMode(MOTOR_2_B,OUTPUT);
      pinMode(MOTOR_3_A,OUTPUT);
      pinMode(MOTOR_3_B,OUTPUT);
      pinMode(MOTOR_4_A,OUTPUT);
      pinMode(MOTOR_4_B,OUTPUT);
      
    moveBot(MOVDIR_STOP);
  }
  
  void update()
  {
//    liveMotorState = liveMotorState == motorState ? MOVDIR_STOP : motorState;
    counter++;
    liveMotorState = (counter < motorPower) ? motorState : MOVDIR_STOP;
    switch(liveMotorState)
    {
      case MOVDIR_STOP:
         setMotors(MOTOR_DIR_STOP,MOTOR_DIR_STOP,MOTOR_DIR_STOP,MOTOR_DIR_STOP);  
       break;
          
      case MOVDIR_FWD:    
         setMotors(MOTOR_DIR_FWD,MOTOR_DIR_REV,MOTOR_DIR_REV,MOTOR_DIR_FWD);
       break;
       
      case MOVDIR_REV:
         setMotors(MOTOR_DIR_REV,MOTOR_DIR_FWD,MOTOR_DIR_FWD,MOTOR_DIR_REV);
       break;
  
      case MOVDIR_RIGHT:     
         setMotors(MOTOR_DIR_REV,MOTOR_DIR_FWD,MOTOR_DIR_REV,MOTOR_DIR_FWD);
       break; 
       
      case MOVDIR_LEFT:     
         setMotors(MOTOR_DIR_FWD,MOTOR_DIR_REV,MOTOR_DIR_FWD,MOTOR_DIR_REV);
       break;
       
      case MOVDIR_SPINCLOCK:     
         setMotors(MOTOR_DIR_FWD,MOTOR_DIR_FWD,MOTOR_DIR_FWD,MOTOR_DIR_FWD);
       break;
       
      case MOVDIR_SPINANTI:     
        setMotors(MOTOR_DIR_REV,MOTOR_DIR_REV,MOTOR_DIR_REV,MOTOR_DIR_REV);
       break;
       
      case MOVDIR_FWD_AND_SPINCLOCK:    
         setMotors(MOTOR_DIR_FWD,MOTOR_DIR_REV,MOTOR_DIR_FWD,MOTOR_DIR_FWD);
       break;

      case MOVDIR_FWD_AND_SPINANTI:    
         setMotors(MOTOR_DIR_FWD,MOTOR_DIR_REV,MOTOR_DIR_REV,MOTOR_DIR_REV);
       break;
       
    }
    
  }

  void moveBot( byte move_dir)
  {
    motorState = move_dir;
//    liveMotorState = move_dir;
  }
  
  void setMotorPower(unsigned char power)
  {
    motorPower = power;    
  }

  void testMotors()
  {
    // Use this to test the motors out of state machine
     digitalWrite(MOTOR_1_A,HIGH); delay(1000); digitalWrite(MOTOR_1_A,LOW);
     digitalWrite(MOTOR_1_B,HIGH); delay(1000); digitalWrite(MOTOR_1_B,LOW);
     digitalWrite(MOTOR_2_A,HIGH); delay(1000); digitalWrite(MOTOR_2_A,LOW);
     digitalWrite(MOTOR_2_B,HIGH); delay(1000); digitalWrite(MOTOR_2_B,LOW);
     digitalWrite(MOTOR_3_A,HIGH); delay(1000); digitalWrite(MOTOR_3_A,LOW);
     digitalWrite(MOTOR_3_B,HIGH); delay(1000); digitalWrite(MOTOR_3_B,LOW);
     digitalWrite(MOTOR_4_A,HIGH); delay(1000); digitalWrite(MOTOR_4_A,LOW);
     digitalWrite(MOTOR_4_B,HIGH); delay(1000); digitalWrite(MOTOR_4_B,LOW);
  }


  void testAllStates()
  {
    if ( (millis()-lastMotorStateUpdateTime) > 1000 )
    {
  
  // test all motor states one by one
        motorState++;
        if (motorState > 6)
        {
            motorState = 1;
        }
        moveBot(motorState);
    }
  }  
          
  void moveRandom()
  {
    if ( (millis()-lastMotorStateUpdateTime) > 1000 )
    {
    // move to random states
        moveBot(random(1,6));
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
  
  
  byte motorState;
  byte liveMotorState;
  unsigned char counter;
  unsigned char motorPower;
  
  unsigned long lastMotorStateUpdateTime;
  
};


#endif
