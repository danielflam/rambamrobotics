rambamrobotics
==============

Overview
--------

This repo contains some arduino helper files to build applications that interface with multiple 
I2C sensors and handle the multi programming.

The idea is that each sensor can be set to be called at a different minimum interval allowing other tasks to complete.

It is based on the high performance I2C library by Wayne Truchsess. See http://dsscircuits.com/articles/arduino-i2c-master-library.html
And you can download it at https://github.com/rambo/I2C

Another useful class is the stopwatch which allows you to track time between events without having to pepper the code
with "if((msecs()-last)>interval)" and have to maintain mutiple variables.


Usage
-----

In genral this is the format of the class:

```cpp
class MyPeripheral : public I2CPeripheral
{
  public:

    MyPeripheral(int interval = SOME_DEFAULT_INTERVAL)
    {
      // This is the interval the I2C bus will be available - provided it is not being used
      // bt another peripheral
      setInterval(interval);
    }
    
      
    void setup()
    {
	// setup your peripheral here
    }

    void update()
    {
      if (lockBus() == I2CPERIPHERAL_LOCKED)
      {
	  // We locked the bus now we can use I2C calls or Wire calls for that matter!
	  //I2c.dosomethinghere(....);

	  //Serial.print("Bus locked: ");
          //Serial.println(millis());
          

	  // When the I2C data aquisition is completed, it signals to the caller that new data is available
	  newDataAvailable();
	  // finally the bus is unlocked
          unlockBus();          
      }
    }  
  private:
};
```

The caller can have multiple peripherals connected after the caller completes processing the data it signals the peripheral that it has processed the data:

```cpp
void loop()
{
   bool newDataAquired = false;

   peripheral1.update();
   if (peripheral1.hasNewData())
   {
      //do something here
      peripheral1.dataProcessed();
      newDataAquired = true;
   }

   peripheral2.update();
   if (peripheral2.hasNewData())
   {
      //do something here
      peripheral2.dataProcessed();
      newDataAquired = true;
   }

   peripheral3.update();
   if (peripheral3.hasNewData())
   {
      //do something here
      peripheral3.dataProcessed();
      newDataAquired = true;
   }

   if (newDataAquired)
   {
	// do something here like recalculate bearings
   }

   // do stuff that always need to be done
}
```


Notes
-----

that this version is *blocking* which means that the I2C calls block while waiting for response - with the following consequences:
* One wants to minimize calls to the sensors
* Make short I2C calls

The included sensor examples are WiiMote Pixart camera and HMC5883 magnetic sensor. More sensors to come!
Both can run at 400KHz fast I2c


