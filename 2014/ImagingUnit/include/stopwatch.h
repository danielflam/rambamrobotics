#ifndef _STOPWATCH_H
#define _STOPWATCH_H


#define STOPWATCH_NORESTART false
#define STOPWATCH_RESTART true

class Stopwatch
{
  public:
     Stopwatch(unsigned long interval = 1000 )
     {
       defaultInterval = interval;
       reset();
     }
     
     void setInterval(unsigned long interval)
     {
       defaultInterval = interval;
     }
     
     unsigned long getInterval()
     {
       return defaultInterval;
     }
     
     void reset()
     {
       last = millis();
     }
     

     bool timeout(bool restart = STOPWATCH_NORESTART)
     {
       current = millis();
       if (current > (last + defaultInterval))
       {
         if (restart)
         {
           last = current;
         }
         return true;
       }
       return false;
     }
     
	 unsigned long timeSinceReset()
	 {
		return millis() - last;
	 }
	 
     
  private:
     unsigned long current;
     unsigned long last;
     long defaultInterval;
    
};

#endif
