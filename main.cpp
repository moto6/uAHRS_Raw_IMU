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
void setLEDPeriod(uint16_t period_ms);
float getDeltaT();

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

		if(MAG[3] == 1){
			float angle = atan2(-MAG[1], -MAG[0])*RAD2DEG;
			if(angle <0){angle += 360.0f;}
			sprintf(ch, "%3d", (int32_t)angle);
			Serial.println(ch);
		}

		// Delta T를 계산한다.
		float dt = getDeltaT();
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

	NVIC_DisableIRQ(TIMER4_IRQn);

	// TIMER 설정
	setLEDPeriod(200);
}

void setLEDPeriod(uint16_t period_ms){
	NRF_TIMER4->TASKS_STOP	 = 1;
	NRF_TIMER4->TASKS_CLEAR  = 1;
	NRF_TIMER4->MODE		 	 = TIMER_MODE_MODE_Timer;
	NRF_TIMER4->BITMODE		 = TIMER_BITMODE_BITMODE_32Bit;
	NRF_TIMER4->PRESCALER	 = 9;
	NRF_TIMER4->CC[0]		 		 = (float)31250 * ((float)period_ms / 1000.0f);
	NRF_TIMER4->INTENSET	 	 = 65536;
	NRF_TIMER4->SHORTS		 	 = 0x01;
	NVIC_EnableIRQ(TIMER4_IRQn);
	NRF_TIMER4->TASKS_START  = 1;
}

void TIMER4_IRQHandler(void){
	static bool toggle = 0;

	if(NRF_TIMER4->EVENTS_COMPARE[0] == 1){
		if(toggle == 0){
			digitalWrite(LED_BT, LOW);
			toggle = 1;
		}else if(toggle == 1){
			digitalWrite(LED_BT, HIGH);
			toggle = 0;
		}
		NRF_TIMER4->EVENTS_COMPARE[0] = 0;
	}
}

float getDeltaT(){
	static float output = 0;
	static uint32_t timePrev = 0;
	uint32_t timeNow = millis();
	output = (timeNow - timePrev) / 1000.0f;
	timePrev = timeNow;
	return output;
}
