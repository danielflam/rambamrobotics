#ifndef _SOCCER_SERVO_H
#define _SOCCER_SERVO_H


template<byte b>
struct BytevlTToType { 
  static const byte value = b; 
};

template < byte pin >
class SoccerServo
{
public:

  SoccerServo() 
  {
    angle = 1;
    servoScandir = 4;
    // any faster and the system crashes
    stopwatch.setInterval(50);
    MinVal = 1;
    MaxVal = 180;
  }

  void setup(float initialAngle, float minval = 1, float maxval = 180)
  {
    servo.attach(pin);
    angle = initialAngle;
    setConstraints(minval,maxval);
  }

  void setConstraints(float minval, float maxval)
  {
    MinVal = minval;
    MaxVal = maxval;
    set(angle);
  }
  
  void set(float newValue)
  {
    servoScandir = newValue > angle ? 7 : -7;    
    angle = newValue;
    angle = constrain(angle, MinVal, MaxVal);
    servo.write(angle);
  }

  void add(float newValue)
  {
    angle += newValue;
    angle = constrain(angle, MinVal, MaxVal);

    servo.write(angle);  
  }



  float get()
  {
    return angle;
  }

  void scan(float minangle = 90, float maxangle = 120 )
  {  
    if (stopwatch.timeout( STOPWATCH_RESTART ))
    {
      if (angle > maxangle)
      {
        servoScandir = -7;    
      }
      else if (angle < minangle)
      {
        servoScandir = 7;
      }
      angle += servoScandir;
      //            angle = constrain(angle,minangle,maxangle);
      servo.write(angle);
      //log(minangle, maxangle);
      //            stopwatch.reset();
    }
  }

  void log()
  {
    Serial.print("setting servo to ");
    Serial.print(angle );
    Serial.print(" min=");
    Serial.print(MinVal );
    Serial.print(" max=");
    Serial.print(MaxVal );
    Serial.print(" step=");
    Serial.print(servoScandir);

    Serial.println("");
  }

private:
  Servo servo;
  Stopwatch stopwatch;
  float angle;  
  int servoScandir;
  int MinVal, MaxVal;
};


#endif

