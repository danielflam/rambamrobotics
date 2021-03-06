#ifndef _I2C_PERIPHERAL_H
#define _I2C_PERIPHERAL_H

#include <I2C.h>


#define I2CPERIPHERAL_UNLOCKED 1
#define I2CPERIPHERAL_LOCKED 2
#define I2CPERIPHERAL_GET 3
#define I2CPERIPHERAL_SET 4
#define I2CPERIPHERAL_ERROR_LOCKED 5
#define I2CPERIPHERAL_ERROR_UNKNOWN_REQUEST 6
#define I2CPERIPHERAL_ERROR_TIMER_BUSY 7
#define I2CPERIPHERAL_ERROR_NO_LOCK_AQUIRED 8


class I2CPeripheral
{
  public:
    I2CPeripheral()
    {
      hasNewDataAvaiable = false;
      lockAquired = false;
    }


    void setInterval(unsigned long interval)
    {
      stopwatch.setInterval(interval);
    }
    
    bool busy()
    {
      return getMutex();
    }
    
    bool hasNewData()
    {
      return hasNewDataAvaiable;
    }  

    void newDataProcessed()
    {
      hasNewDataAvaiable = false;
    }  

	void update()
	{
		if (lockBus() == I2CPERIPHERAL_LOCKED)
		{
			onUpdate();
			unlockBus();
		}
	}

	// User must implement this function
	virtual void onUpdate() = 0;

    
  protected:

    int lockBus()
    {
      if (!stopwatch.timeout(STOPWATCH_NORESTART))
      {
        return I2CPERIPHERAL_ERROR_TIMER_BUSY;
      }

      bool mutex = getMutex();

      if (mutex)
      {
        return I2CPERIPHERAL_ERROR_LOCKED;
      }      
            
      lockAquired = true;        
      mutex = true;
      stopwatch.reset();
      
      return I2CPERIPHERAL_LOCKED;
    }
    
    int unlockBus()
    {
//        if (!lockAquired)
//        {
//          return I2CPERIPHERAL_ERROR_NO_LOCK_AQUIRED;
//        }

        bool mutex = getMutex();
  
        if (!mutex)
        {
          return I2CPERIPHERAL_ERROR_NO_LOCK_AQUIRED;
        }      
        
        mutex = false;
        lockAquired = false;
        stopwatch.reset();
        return I2CPERIPHERAL_UNLOCKED;
    }

    void newDataAvailable()
    {
      hasNewDataAvaiable = true;
    }  

  // How much time between each peripheral access?
    unsigned long peripheralDelayTime;
    bool lockAquired;    
    bool hasNewDataAvaiable;    
  private:

    Stopwatch stopwatch;
    
    static bool & getMutex()
    {
      static bool mutex = false;
      
      return mutex;
    }
};

#endif

