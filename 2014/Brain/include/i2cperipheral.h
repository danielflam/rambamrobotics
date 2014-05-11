#ifndef _I2C_PERIPHERAL_H
#define _I2C_PERIPHERAL_H

//#include <I2C.h>

#include <stopwatch.h>


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
    I2CPeripheral(unsigned long delayTime = 500)
    {
      peripheralDelayTime = delayTime;
      hasDataAvaiable = false;
      lockAquired = false;
    }

    virtual ~I2CPeripheral(){}
    void setInterval(unsigned long interval)
    {
      stopwatch.setInterval(interval);
    }
    
    bool busy()
    {
      return getMutex();
    }

    
    bool dataAvailable()
    {
      return hasDataAvaiable;
    }  

    void clearDataAvailableDataProcessed()
    {
      hasDataAvaiable = false;
    }  

	void update()
	{
		if (lockBus() == I2CPERIPHERAL_LOCKED)
		{
			hasDataAvaiable = hasDataAvaiable || onUpdate();
			unlockBus();
		}
	}



  protected:

	// User must implement this function
	// returns true if new data is available as a result of operation
	virtual bool onUpdate() = 0;

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

    void setDataAvailable()
    {
      hasDataAvaiable = true;
    }  

  // How much time between each peripheral access?
    unsigned long peripheralDelayTime;
    bool lockAquired;    
    bool hasDataAvaiable;
  private:

    Stopwatch stopwatch;

    // for single device
    static bool & getMutex()
    {
      static bool mutex = false;
      
      return mutex;
    }
};

#endif

