rambamrobotics
==============

Overview
--------

This is the 2012 code rewrite for the soccer bots. Our soccer bots run using 
* Arduino
* A custom made soccerbot shield (details will follow)
* Custom made hardware
* Custom wii-pixart sensor board mounted on servo
* sb101 based bluetooth telemetry
* PDM (not PWM) based motor control for smoother and more efficient motor control

We are planning on building a few of these in order to introduce robotic principles to kids. 

StopWatch
==========
a useful class is the stopwatch which allows you to track time between events without having to pepper the code
with "if((msecs()-last)>interval)" and have to maintain mutiple variables.



i2cperipheral
==============

Overview
--------

This repo contains some arduino helper files to build applications that interface with multiple 
I2C sensors and handle the multi programming. 

The idea is that each sensor can be set to be called at a different minimum interval allowing other tasks to complete.

It is based on the high performance I2C library by Wayne Truchsess. See http://dsscircuits.com/articles/arduino-i2c-master-library.html
And you can download it at https://github.com/rambo/I2C



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

    void onUpdate()
    {
	  // When this call is made, the i2c is guaranteed to be available exclusively to this peripheral
	  // So try to do it as fast as possible! 
	  //I2c.dosomethinghere(....);

	  //Serial.print("Bus locked: ");
          //Serial.println(millis());

	  // When the I2C data aquisition is completed, it signals to the caller that new data is available
	  newDataAvailable();
      }
    }  
  private:
};
```

The caller can have multiple peripherals connected after the caller completes processing the data it signals the peripheral that it has processed the data:

```cpp

// IN main file:

bool newDataAquired;

void loop()
{
   newDataAquired = false;
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

Improvement
-----------

A further improvement would be to create peripheral sub-classes where the new data is processed and the main loop 
could be de-clutterfied.

```cpp

extern bool newDataAquired;

class MyHMC5883 : public HMC5883
{
	MyHMC5883() : 
	   HMC5883()
	{
		
	}

	void doUpdate()
	{
		HMC5883::doUpdate();
		if (hasNewData())
		{
			// Do something here

			// if you call next line main loop will not know about new data
			dataProcessed();
			// so we signal main loop that something changed!
			newDataAquired = true;
		}
	}
}

```


Notes
-----

that this version is *blocking* which means that the I2C calls block while waiting for response - with the following consequences:
* One wants to minimize calls to the sensors
* Make short I2C calls

The included sensor examples are WiiMote Pixart camera and HMC5883 magnetic sensor. More sensors to come!
Both can run at 400KHz fast I2c


This repo is the seed to a better shared peripheral library to be announced