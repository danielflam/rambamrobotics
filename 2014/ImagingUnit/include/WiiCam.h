#ifndef _WIICAM_H
#define _WIICAM_H

// Wii Remote IR sensor  test sample code  by kako
// modified output for Wii-BlobTrack program by RobotFreak
#include <i2cperipheral.h>
#include <kalman.h>

#define WIICAM_SLAVE_ADDRESS_7BIT 0x58
// 8 bit address
#define WIICAM_SLAVE_ADDRESS 0xB0

class WiiCam : public I2CPeripheral<WIICAM_SLAVE_ADDRESS>
{

public:
    WiiCam(unsigned long delayTime = 10) :
    	I2CPeripheral<WIICAM_SLAVE_ADDRESS>(delayTime)
    {
//      setInterval(10); // wii detect interval
        setInterval(10); // wii detect interval
        lastDetected.setInterval(75); // how long before we lose sight of ball

        for (i=0; i<3; i++)
        {
          Ix[i] = 0;
          Iy[i] = 0;
          avgIx[i] = 0;
          avgIy[i] = 0;
        }
        avgX = 0;
        avgY = 0;
  	  hastimedout = false;
      }
      
      unsigned int Ix[4];
      unsigned int Iy[4];

      unsigned int avgIx[4];
      unsigned int avgIy[4];

      unsigned int avgX;
      unsigned int avgY;
    
    void checkRes(const char * s, bool res)
    {
    	if (!res)
    	{
    		Serial.print(s);
    		Serial.println(" failed.");
    	}
    }

    typedef uint8_t Sequence[2];

	void setup()
    {
		Sequence sequences[] = {
			    {0x30,0x01},
			    {0x30,0x08},
			    {0x06,0x90},
			    {0x08,0xC0},
			    {0x1A,0x40},
			    {0x33,0x33}
		};


//        const uint8_t sequence1[]= { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09 };
//        const uint8_t sequence2[]= { 0x00, 0x41 };
//        const uint8_t sequence3[]= { 0x40, 0x00 };

        Serial.println("Initializing PixArt Camera...");

        for (int i=0; i<6;i++)
        {
        	checkRes("Sequence init", write(sequences[i], 2));
        	delay(10);
        }
        // MUST BE BLOCKING CALLS
        
//        checkRes("write to reg 30", write(0x30,0x01));
//		delay(30);
//		checkRes("reg 00 sequence 1", write(0x00,const_cast<uint8_t*>(sequence1), sizeof(sequence1)));
//        delay(30);
//        checkRes("reg 07 sequence 2", write(0x07,const_cast<uint8_t*>(sequence2), sizeof(sequence2)));
//        delay(30);
//        checkRes("reg 1A sequence 3", write(0x1A,const_cast<uint8_t*>(sequence3), sizeof(sequence3)));
//        delay(30);
//        checkRes("reg 33", write(0x33,0x33));
//        delay(30);
//        checkRes("reg 30", write(0x30, 0x08));
//        Serial.println("PixArt Camera Initialized");
//        delay(100);
    }
	

    virtual bool onUpdate()
    {
        if (read(0x36, (uint8_t*)data_buf, 16) == 0)
        {
        	setPeripheralError(true);
        	return hasDataAvaiable;
        }

        setPeripheralError(false);

        byte j;
        int found = 0;
        unsigned int tmpX = 0;
        unsigned int tmpY = 0;
        
        for (i = 0; i<4; i++)
        {
          j = 3*i+1;
          Ix[i] = data_buf[j];
          Iy[i] = data_buf[j+1];
          s   = data_buf[j+2];
          Ix[i] += (s & 0x30) <<4;
          Iy[i] += (s & 0xC0) <<2;
          
          if ( Ix[i] >0 && Ix[i] < 1023 && Iy[i] >0 && Iy[i] < 1023)
          {
            found++;
            avgIx[i] = (avgIx[i] + Ix[i]) >> 1;
            avgIy[i] = (avgIx[i] + Ix[i]) >> 1;
            tmpX += Ix[i];
            tmpY += Iy[i];
          }
        }
        
        if (found > 0)
        {
		  hastimedout = false;
          lastDetected.reset();
          //found += 1;
          //avgX = (avgX + tmpX)/found;
          //avgY = (avgY + tmpY)/found;
          avgX = tmpX/found;
          avgY = tmpY/found;
		  
          setDataAvailable();
        } else if (!hastimedout && lastDetected.timeout())
        {
		  hastimedout = true;
          avgX = 0;
          avgY = 0;
          for (i = 0; i<4; i++)
          {
            avgIx[i] = 0;
            avgIy[i] = 0;
          }
		  setDataAvailable();
        }

        return hasDataAvaiable;
    }  

    
    void log()
    {  
		if (isPeripheralError())
		{
			Serial.println("I2C error reading Pixart Camera");

			return;
		}

		for (i=0; i<16; i++)
		{
			Serial.print(data_buf[i], HEX);
			Serial.print(" ");
		}
		Serial.println();

        for(i=0; i<4; i++)
        {
          printNum(Ix[i]);
          Serial.print(",");
          printNum(Iy[i]);
          Serial.print(",");
        }
        Serial.print("||,");
        for(i=0; i<4; i++)
        {
          printNum(avgIx[i]);
          Serial.print(",");
          printNum(avgIy[i]);
          Serial.print(",");
        }
        Serial.print("||,");
        printNum(avgX);
        Serial.print(",");
        printNum(avgY);
        
        Serial.println("");
   }
    
  private:



	void printRes(char * str, uint8_t res)
    {
      Serial.print(str);
      Serial.println(res, HEX);


    }  
  
    void printNum(int num)
    {
          if (num < 1000)
            Serial.print(" ");
          if (num < 100)  
            Serial.print(" ");
          if (num < 10)  
            Serial.print(" ");
          Serial.print(num);
    }

	Stopwatch lastDetected;
    byte data_buf[16];
    int i;
    
    int s;
	bool hastimedout;
};

#endif
