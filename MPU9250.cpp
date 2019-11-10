/*
 * Sources for MPU9250 IMU Sensor
 * Creator : Seonguk Jeong
 * Last Updated : Nov. 11, 2019
 */

#include "MPU9250.h"
void Init();
uint8_t _MPU9250::getDeviceId(){
	static uint8_t output = 0;
	Wire.beginTransmission(MPU9250);
	Wire.write(WHO_AM_I);
	Wire.endTransmission(false);
	Wire.requestFrom(MPU9250, 1);
	output = Wire.read();
	Wire.endTransmission(true);
	return output;
}
uint8_t _MPU9250::getMagId(){

}
int16_t _MPU9250::getRawAccel(){

}
int16_t _MPU9250::getRawGyro(){

}
int16_t _MPU9250::getRawMag(){

}
void _MPU9250::caliAccel(){

}
void _MPU9250::caliGyro(){

}
void _MPU9250::caliMag(){

}
