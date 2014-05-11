/*
 * rambamIMU.cpp
 *
 *  Created on: Apr 6, 2014
 *      Author: Danny
 *      Based on:
 *      https://github.com/mikeshub/Razor_10736_Open_IMU/blob/master/Razor_10736_Open_IMU/Razor_10736_Open_IMU.ino
 *      and FreeIMU
 */


#include <Arduino.h>
#include <rambamIMU.h>
#include <Wire.h>
#include <math.h>

#define ToRad(x) ((x) * 0.01745329252)  // *pi/180
#define ToDeg(x) ((x) * 57.2957795131)  // *180/pi
#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f
#define betaDef		0.15f


#define ADXL435_ADDR 0x53
#define ITG3200_ADDR 0x68
#define HMC5883_ADDR 0x1E
//accel defines
#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define FIFO_CTL 0x38
#define DATAX0 0x32
//gyro defines
#define POWER_MGMT 0x3E
#define SRD 0x15
#define DLPF 0x16
#define GYRO_XOUT_H 0x1D
#define GYRO_TO_RAD 0.001214142f //(14.375^-1 * pi/180)
#define GYRO_TO_DEG 0.069565217f
//mag defines
#define CONFIG_REG_A (uint8_t)0x00
#define CONFIG_REG_B 0x01
#define MODE_REG 0x02
#define MAG_X_MSB 0x03

#define compassXMax 327.0f
#define compassXMin -284.0f
#define compassYMax 280.0f
#define compassYMin -334.0f
#define compassZMax 343.0f
#define compassZMin -267.0f
#define inverseXRange (float)(2.0 / (compassXMax - compassXMin))
#define inverseYRange (float)(2.0 / (compassYMax - compassYMin))
#define inverseZRange (float)(2.0 / (compassZMax - compassZMin))


void RambamIMU::setup(){
 // TWBR = ((F_CPU / 400000) - 16) / 2;
  Init();
  timer = micros();
}

void RambamIMU::loop(){
  if (millis() - timer >= 5){
    dt = float(millis() - timer)/1000.0;
    timer=millis();
    GetMag();
    GetGyro();

    GetAcc();
    AHRSupdate();
    loopCount = 0;
  }
}

void RambamIMU::print()
{
	  if (millis() - printTimer > 50){
	    printTimer = millis();
	    GetEuler();
	    Serial.print("$YPR");
	    Serial.print(",");
	    Serial.print(yaw);
	    Serial.print(",");
	    Serial.print(pitch);
	    Serial.print(",");
	    Serial.print(roll);
	    Serial.println("*");
	  }
}

void RambamIMU::writeReg(int addr, int reg, int val)
{
	  Wire.beginTransmission(addr);
	  Wire.write(reg);
	  Wire.write(val);
	  Wire.endTransmission();
}

void RambamIMU::Init(){
 writeReg(HMC5883_ADDR, CONFIG_REG_A, 0x1C);//220Hz update rate
 writeReg(HMC5883_ADDR, CONFIG_REG_B,0x60);//+/- 2.5 gauss
 writeReg(HMC5883_ADDR, MODE_REG, (uint8_t)0x00);//continuous conversion mode

 writeReg(ADXL435_ADDR,BW_RATE,0x0C);
  delay(5);
  writeReg(ADXL435_ADDR,POWER_CTL,0x08);
  delay(5);
  writeReg(ADXL435_ADDR,DATA_FORMAT,0x0B);

  writeReg(ITG3200_ADDR, POWER_MGMT, 0x80);
  delay(5);
  writeReg(ITG3200_ADDR, DLPF,0x18);
  delay(5);

  writeReg(ITG3200_ADDR, SRD, HEX_ZERO);
  delay(5);

  writeReg(ITG3200_ADDR, POWER_MGMT, 0x03);

  gyroSumX = 0;
  gyroSumY = 0;
  gyroSumZ = 0;
  for (int i =0; i<512; i++){
    GetGyro();//take samples to let the internal LPFs work
    GetAcc();
    delay(1);
  }
  for (int i =0; i<512; i++){
    GetGyro();
    GetAcc();
    gyroSumX += gyroX;
    gyroSumY += gyroY;
    gyroSumZ += gyroZ;
    delay(1);
  }
  offsetX = gyroSumX >> 9;
  offsetY = gyroSumY >> 9;
  offsetZ = gyroSumZ >> 9;

  GetMag();
  GetAcc();
  //calculate the initial quaternion
  //these are rough values. This calibration works a lot better if the device is kept as flat as possible
  //find the initial pitch and roll
  pitch = ToDeg(fastAtan2(smoothX(),sqrt(smoothY() * smoothY() + smoothZ() * smoothZ())));
  roll = ToDeg(fastAtan2(-1*smoothY(),sqrt(smoothX() * smoothX() + smoothZ() * smoothZ())));

  if (smoothZ() > 0){
    if (smoothX() > 0){
      pitch = 180.0 - pitch;
    }
    else{
      pitch = -180.0 - pitch;
    }
    if (smoothY() > 0){
      roll = -180.0 - roll;
    }
    else{
      roll = 180.0 - roll;
    }
  }


  //tilt compensate the compass
  float xMag = (floatMagX * cos(ToRad(pitch))) + (floatMagZ * sin(ToRad(pitch)));
  float yMag = -1 * ((floatMagX * sin(ToRad(roll))  * sin(ToRad(pitch))) + (floatMagY * cos(ToRad(roll))) - (floatMagZ * sin(ToRad(roll)) * cos(ToRad(pitch))));
  yaw = ToDeg(fastAtan2(yMag,xMag));

  if (yaw < 0){
    yaw += 360;
  }
  //calculate the rotation matrix
  float cosPitch = cos(ToRad(pitch));
  float sinPitch = sin(ToRad(pitch));

  float cosRoll = cos(ToRad(roll));
  float sinRoll = sin(ToRad(roll));

  float cosYaw = cos(ToRad(yaw));
  float sinYaw = sin(ToRad(yaw));

  //need the transpose of the rotation matrix
  float r11 = cosPitch * cosYaw;
  float r21 = cosPitch * sinYaw;
  float r31 = -1.0 * sinPitch;

  float r12 = -1.0 * (cosRoll * sinYaw) + (sinRoll * sinPitch * cosYaw);
  float r22 = (cosRoll * cosYaw) + (sinRoll * sinPitch * sinYaw);
  float r32 = sinRoll * cosPitch;

  float r13 = (sinRoll * sinYaw) + (cosRoll * sinPitch * cosYaw);
  float r23 = -1.0 * (sinRoll * cosYaw) + (cosRoll * sinPitch * sinYaw);
  float r33 = cosRoll * cosPitch;



  //convert to quaternion
  q0 = 0.5 * sqrt(1 + r11 + r22 + r33);
  q1 = (r32 - r23)/(4 * q0);
  q2 = (r13 - r31)/(4 * q0);
  q3 = (r21 - r12)/(4 * q0);

  float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

}

inline float calcMag(float mag, float compassMin, float inverseRange)
{
	return (mag - compassMin) * inverseRange - 1.0;
}
void RambamIMU::GetMag(){
  uint8_t lsb, msb;

  Wire.beginTransmission(HMC5883_ADDR);
  Wire.write(MAG_X_MSB);
  Wire.endTransmission();
  Wire.beginTransmission(HMC5883_ADDR);
  Wire.requestFrom(HMC5883_ADDR,6);

  msb = Wire.read();//X register
  lsb = Wire.read();
  magY = (((msb << 8) | lsb));

  msb = Wire.read();//Z register
  lsb = Wire.read();
  magZ = -1 * (((msb << 8) | lsb));

  msb = Wire.read();//Y register
  lsb = Wire.read();
  magX = (((msb << 8) | lsb));

  floatMagX = calcMag ((float)magX, compassXMin, inverseXRange);
  floatMagY = calcMag ((float)magY, compassYMin, inverseYRange);
  floatMagZ = calcMag ((float)magZ, compassZMin, inverseZRange);

}
void RambamIMU::GetAcc(){
  uint8_t lsb, msb;

  Wire.beginTransmission(ADXL435_ADDR);
  Wire.write(DATAX0);
  Wire.endTransmission();
  Wire.beginTransmission(ADXL435_ADDR);
  Wire.requestFrom(ADXL435_ADDR,6);
  lsb = Wire.read();
  msb = Wire.read();
  accY = -1 * ((msb << 8) | lsb);
  lsb = Wire.read();
  msb = Wire.read();
  accX = -1 * ((msb << 8) | lsb);
  lsb = Wire.read();
  msb = Wire.read();
  accZ = (msb << 8) | lsb;
  smoothX.update(accX);
  smoothY.update(accY);
  smoothZ.update(accZ);

//  accToFilterX = smoothX();//if the value from the smoothing filter is sent it will not work when the algorithm normalizes the vector
//  accToFilterY = smoothY();
//  accToFilterZ = smoothZ();
}

void RambamIMU::GetGyro(){
  uint8_t lsb, msb;
  Wire.beginTransmission(ITG3200_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission();
  Wire.beginTransmission(ITG3200_ADDR);
  Wire.requestFrom(ITG3200_ADDR,6);

  msb = Wire.read();
  lsb = Wire.read();
  gyroY = (((msb << 8) | lsb) - offsetX);

  msb = Wire.read();
  lsb = Wire.read();
  gyroX = (((msb << 8) | lsb) - offsetY);

  msb = Wire.read();
  lsb = Wire.read();
  gyroZ = -1 * (((msb << 8) | lsb) - offsetZ);

  scaledGyroX = GYRO_TO_RAD * gyroX;
  scaledGyroY = GYRO_TO_RAD * gyroY;
  scaledGyroZ = GYRO_TO_RAD * gyroZ;
}

void RambamIMU::GetEuler(void){
  roll = ToDeg(fastAtan2(2 * (q0 * q1 + q2 * q3),1 - 2 * (q1 * q1 + q2 * q2)));
  pitch = ToDeg(asin(2 * (q0 * q2 - q3 * q1)));
  yaw = ToDeg(fastAtan2(2 * (q0 * q3 + q1 * q2) , 1 - 2* (q2 * q2 + q3 * q3)));
  if (yaw < 0){
    yaw +=360;
  }

}

void RambamIMU::AHRSupdate() {

  gx = scaledGyroX;
  gy = scaledGyroY;
  gz = scaledGyroZ;

  ax = smoothX(); //accToFilterX;
  ay = smoothY(); //accToFilterY;
  az = smoothZ(); //accToFilterZ;

  mx = floatMagX;
  my = floatMagY;
  mz = floatMagZ;
  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  magnitude = sqrt(ax * ax + ay * ay + az * az);

  if ((magnitude > 384) || (magnitude < 128)){
    ax = 0;
    ay = 0;
    az = 0;
  }

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

	  // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= betaDef * s0;
    qDot2 -= betaDef * s1;
    qDot3 -= betaDef * s2;
    qDot4 -= betaDef * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}



float RambamIMU::fastAtan2( float y, float x)
{
  static float atan;
  static float z;
  if ( x == 0.0f )
  {
    if ( y > 0.0f ) return PIBY2_FLOAT;
    if ( y == 0.0f ) return 0.0f;
    return -PIBY2_FLOAT;
  }
  //atan;
  z = y / x;
  if ( fabs( z ) < 1.0f )
  {
    atan = z/(1.0f + 0.28f*z*z);
    if ( x < 0.0f )
    {
      if ( y < 0.0f ) return atan - PI_FLOAT;
      return atan + PI_FLOAT;
    }
  }
  else
  {
    atan = PIBY2_FLOAT - z/(z*z + 0.28f);
    if ( y < 0.0f ) return atan - PI_FLOAT;
  }
  return atan;
}

float RambamIMU::invSqrt(float number) {
	if (number<0)
		return 0.0;
	union {
		  uint32_t i;
		  float y;
	  } u;
	  //volatile unsigned long i;
	  //volatile float x, y;
	  float x;//, y;
	  const float f = 1.5F;

	  x = number * 0.5F;
	  u.y = number;
	  u.i = * ( long * ) &u.y;
	  u.i = 0x5f375a86 - ( u.i >> 1 );
	  u.y = * ( float * ) &u.i;
	  u.y = u.y * ( f - ( x * u.y * u.y ) );
	  return u.y;
}
