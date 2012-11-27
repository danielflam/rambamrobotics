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
        stopwatch.setInterval(50);
      }
      
      void setup(int initialAngle)
      {
        servo.attach(pin);
        angle = constrain(initialAngle,1,180);
        servo.write(angle);
      }
      
      void set(int newValue)
      {
          angle = newValue;
          angle = constrain(angle, 1, 180);
          servo.write(angle);
      }

      void add(int newValue)
      {
          angle += newValue;
          angle = constrain(angle, 1, 180);
          
          servo.write(angle);
      }
      
      

      int get()
      {
        return angle;
      }
      
      void scan(int minangle = 90, int maxangle = 120 )
      {
          if (stopwatch.timeout( STOPWATCH_RESTART ))
          {
            if (angle > maxangle)
            {
              servoScandir = -4;    
            }
            else if (angle < minangle)
            {
              servoScandir = 4;
            }
            angle += servoScandir;
//            angle = constrain(angle,minangle,maxangle);
            servo.write(angle);
            log(minangle, maxangle);
//            stopwatch.reset();
          }
      }
  
      void log(int minangle = 90, int maxangle = 120)
      {
           Serial.print("setting servo to ");
            Serial.print(angle );
            Serial.print(" min=");
           Serial.print(minangle );
            Serial.print(" max=");
            Serial.print(maxangle );
            Serial.print(" step=");
            Serial.print(servoScandir);

            Serial.println("");
      }
      
  private:
      Servo servo;
      Stopwatch stopwatch;
      int angle;  
      int servoScandir;
};


#endif
