#ifndef _HMC5883_H
#define _HMC5883_H

#define HMC5883_ADDRESS 0x1E 

#define HMC5883_ConfigurationRegisterA 0x00
#define HMC5883_ConfigurationRegisterB 0x01
#define HMC5883_ModeRegister 0x02
#define HMC5883_DataRegisterBegin 0x03

#define HMC5883_Measurement_Continuous 0x00
#define HMC5883_Measurement_SingleShot 0x01
#define HMC5883_I2C_HIGHSPEED 0x80
#define HMC5883_Measurement_Idle 0x03


#define HMC5883_RegisterA_AVG_1 B00000000
#define HMC5883_RegisterA_AVG_2 B00100000
#define HMC5883_RegisterA_AVG_4 B01000000
#define HMC5883_RegisterA_AVG_8 B01100000

#define HMC5883_RegisterA_DataRate_0_75HZ   B00000000
#define HMC5883_RegisterA_DataRate_1_5HZ    B00000100
#define HMC5883_RegisterA_DataRate_3HZ      B00001000
#define HMC5883_RegisterA_DataRate_7_5HZ    B00001100
#define HMC5883_RegisterA_DataRate_15HZ     B00010000
#define HMC5883_RegisterA_DataRate_30HZ     B00010100
#define HMC5883_RegisterA_DataRate_75HZ     B00011000
#define HMC5883_RegisterA_DataRate_RESERVED B00011100

#define HMC5883_RegisterA_MeasurmentBias_Normal 0x00
#define HMC5883_RegisterA_MeasurmentBias_Positive 0x01 
#define HMC5883_RegisterA_MeasurmentBias_Negative 0x02

//#define HMC5883_GAUSS_0_88 0
//#define HMC5883_GAUSS_1_3 1
//#define HMC5883_GAUSS_1_9 2
//#define HMC5883_GAUSS_2_5 3
//#define HMC5883_GAUSS_4_0 4
//#define HMC5883_GAUSS_4_7 5
//#define HMC5883_GAUSS_5_6 6
//#define HMC5883_GAUSS_8_1 7

#define HMC5883_GAUSS_0_88 B00000000
#define HMC5883_GAUSS_1_3  B00100000
#define HMC5883_GAUSS_1_9  B01000000
#define HMC5883_GAUSS_2_5  B01100000
#define HMC5883_GAUSS_4_0  B10000000
#define HMC5883_GAUSS_4_7  B10100000
#define HMC5883_GAUSS_5_6  B11000000
#define HMC5883_GAUSS_8_1  B11100000


class HMC5883 : public I2CPeripheral
{
  public:
    HMC5883(int interval = 68)
    {
       setInterval(interval);
	   m_Scale = 0.92;
	}

	// Yes C++ ugly - but efficient for arduino
    int X, Y, Z;
	int Xmin, Xmax, Ymin, Ymax;
    int avgX, avgY, avgZ;
	float minheading;
	float maxheading;
   
    void setup()
    {
		Xmin = Ymin = 30;
		Xmax = Ymax = 230;			
		minheading = 40;
		maxheading = 70;
        Serial.println("Starting up HMC5883...");
        delay(50);
		I2c.write(HMC5883_ADDRESS, HMC5883_ConfigurationRegisterA, HMC5883_RegisterA_DataRate_15HZ | HMC5883_RegisterA_AVG_4 );
        delay(50);
		I2c.write(HMC5883_ADDRESS, HMC5883_ConfigurationRegisterB, HMC5883_GAUSS_0_88  );// 
        delay(50);
        I2c.write(HMC5883_ADDRESS, HMC5883_ModeRegister, HMC5883_Measurement_Continuous  );// continuous mode 
        delay(67);
//        I2c.write(HMC5883_ADDRESS, HMC5883_DataRegisterBegin);
        Serial.println("HMC5883 Initialized");
        avgX=0;
        avgY=0;
        avgZ=0;
    }

//	int SetScale(int gauss)
//	{
//		uint8_t regValue = 0x00;
//		switch (gauss)
//		{
//			case HMC5883_GAUSS_0_88:
//				regValue = 0x00;
//				m_Scale = 0.73;
//				break;
//			case HMC5883_GAUSS_1_3 :
//				regValue = 0x01;
////				m_Scale = 0.92;
//				m_Scale = 1.3;
//				break;
//			case HMC5883_GAUSS_1_9 :
//				regValue = 0x02;
//				m_Scale = 1.22;
//				break;
//			case HMC5883_GAUSS_2_5 :
//				regValue = 0x03;
//				m_Scale = 1.52;
//				break;
//			case HMC5883_GAUSS_4_0 :
//				regValue = 0x04;
//				m_Scale = 2.27;
//				break;
//			case HMC5883_GAUSS_4_7 :
//				regValue = 0x05;
//				m_Scale = 2.56;
//				break;
//			case HMC5883_GAUSS_5_6 :
//				regValue = 0x06;
//				m_Scale = 3.03;
//				break;
//			case HMC5883_GAUSS_8_1 :
//				regValue = 0x07;
//				m_Scale = 4.35;
//				break;
//		}		
//		
//		// Setting is in the top 3 bits of the register.
//        I2c.write(HMC5883_ADDRESS, HMC5883_ConfigurationRegisterB, regValue << 5);// 
//        delay(50);
//        I2c.write(HMC5883_ADDRESS, HMC5883_DataRegisterBegin);
//
//	}
    virtual void onUpdate()
    {
         // Read data from the sensor
         
         I2c.read(HMC5883_ADDRESS, 6, buf);

         X = ( (int(buf[0]) << 8) | buf[1]) ;
         Z = ( (int(buf[2]) << 8) | buf[3]);
         Y = ( (int(buf[4]) << 8) | buf[5]);

//		if (Xmin > X)
//			Xmin = X;
//		if (Xmax < X)
//			Xmax = X;
//		if (Ymin > Y)
//			Ymin = Y;
//		if (Ymax < Y)
//			Ymax = Y;

		
//		X -= 127;
//		Y -= 127;

         avgX = (avgX + X) >> 1; // divide by 2
         avgY = (avgY + Y) >> 1; // divide by 2
         avgZ = (avgZ + Z) >> 1; // divide by 2


		 I2c.write(HMC5883_ADDRESS, 0x03);

		 
         newDataAvailable();
    }  

	float heading()
	{
			

		
		float heading = atan2(float(X), float(Y));
		//= atan2((double)Y,(double)X) * (180 / 3.14159265) + 180;

		
		

		// Correct for when signs are reversed.
		if(heading < 0) heading += 2*PI;
		if(heading > 2*PI) heading -= 2*PI;

		heading *= RAD_TO_DEG;

//		if (minheading > heading)
//		{
//			minheading = heading;
//		}
//		else if (maxheading < heading)
//		{
//			maxheading = heading;
//		}
		
		return heading; //map(heading, 30, 80, 0,359.99); 

	}

	void log()
	{
         Serial.print("HMC5883: ");
         for (int i=0;i<6;i++)
         {
           Serial.print(buf[i], HEX);
           Serial.print(" ");           
         }
         Serial.print("\t| X: ");           
         Serial.print(X);           
         Serial.print(" Y: ");           
         Serial.print(Y);           
         Serial.print(" Z: ");           
         Serial.print(Z);           
//         Serial.print("\t|Xmin: ");           
//         Serial.print(Xmin);           
//         Serial.print(" Xmax: ");           
//         Serial.print(Xmax);           
//         Serial.print(" Ymin: ");           
//         Serial.print(Ymin);           
//         Serial.print(" Ymax: ");           
//         Serial.print(Ymax);           
		 Serial.print("\t Heading: ");
		 Serial.print(heading());
         Serial.println();
	}
  
  private:
	  float m_Scale;
  	  byte buf[6];

};


#endif

