#ifndef _I2C_TEST_PERIPHERAL_H
#define _I2C_TEST_PERIPHERAL_H

// Wii Remote IR sensor  test sample code  by kako
// modified output for Wii-BlobTrack program by RobotFreak


class I2CTestPeripheral : public I2CPeripheral
{
  public:
    I2CTestPeripheral(int interval)
    {
      setInterval(interval);
    }
    
      
    void setup()
    {
    }

    void read()
    {
      if (lockBus() == I2CPERIPHERAL_LOCKED)
      {
          Serial.print("Bus locked: ");
          Serial.println(millis());
          
          newDataAvailable();
          
          unlockBus();          
          Serial.print("Bus unlocked: ");
          Serial.println(millis());
      }
    }  

    
  private:
  
};

#endif
