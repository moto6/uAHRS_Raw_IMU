#include "Arduino.h"
#include "MPU9250.h"
#include "AHRS.h"
#include "Bluetooth.h"
#include "nrf_drv_timer.h"
#include <math.h>

#define LED_SYS 23
#define LED_BT	24

#define RAD2DEG 	180.0f/PI
#define DEG2RAD 	PI/180.0f

extern _Serial Serial;
extern _BT BT;
_MPU9250 IMU;

void Initialize();

int main(){
	// 모듈 초기화
	Initialize();
	arduinoInit();
	IMU.Init();
	BT.Init();

	// 메인 루프 시작
	char ch[40];
	int16_t *ACCEL, *GYRO, *MAG;

	// 시스템 상태 LED를 켠다
	digitalWrite(LED_SYS, LOW);
	while(1){
		// UART 데이터를 읽는다
		uint8_t command = Serial.read();
		if(command == 0x31){
			Serial.println(" >>> Start Accelerometer Calibration");
			IMU.caliAccel();
			NVIC_SystemReset();
		}else if(command == 0x32){
			Serial.println(" >>> Start Gyroscope Calibration");
			IMU.caliGyro();
			NVIC_SystemReset();
		}else if(command == 0x33){
			Serial.println(" >>> Start Magnetometer Calibration");
			IMU.caliMag();
			NVIC_SystemReset();
		}

		// Raw data를 읽어온다
		ACCEL = IMU.getRawAccel();
		GYRO = IMU.getRawGyro();
		MAG = IMU.getCalibratedMag();

		sprintf(ch, "%d,%d,%d", ACCEL[0], ACCEL[1], ACCEL[2]);
		Serial.print(ch);
		sprintf(ch, ",%d,%d,%d", GYRO[0], GYRO[1], GYRO[2]);
		Serial.print(ch);
		sprintf(ch, ",%d,%d,%d", MAG[0], MAG[1], MAG[2]);
		Serial.println(ch);
	}

	return 0;
}

void Initialize(){
	// UART 통신 설정
	Serial.begin(115200);
	char ch[40];
	Serial.println(" >>> Roverdyn AHRS v1.00.00");
	Serial.println(" >>> Initialize Device");

	// LED 설정
	for(uint8_t i=0;i<2;i++){
		pinMode(LED_SYS + i, OUTPUT);
		digitalWrite(LED_SYS + i, HIGH);
	}

	// Clock 설정
	NRF_CLOCK->TASKS_HFCLKSTART = 1;
	while(!NRF_CLOCK->EVENTS_HFCLKSTARTED){}
	NRF_CLOCK->TASKS_LFCLKSTART = 1;
	while(!NRF_CLOCK->EVENTS_LFCLKSTARTED){}

}
