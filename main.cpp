#include "Arduino.h"
#include "MPU9250.h"
#include "AHRS.h"
#include "Bluetooth.h"
#include "nrf_timer.h"

#define LED_SYS 23
#define LED_BT	24

extern _Serial Serial;

void Initialize();



int main(){
	Initialize();
	while(1){

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
	NRF_TIMER4->TASKS_STOP	 = 1;
	NRF_TIMER4->TASKS_CLEAR  = 1;
	NRF_TIMER4->MODE		 = TIMER_MODE_MODE_Timer;
	NRF_TIMER4->BITMODE		 = TIMER_BITMODE_BITMODE_32Bit;
	NRF_TIMER4->PRESCALER	 = 9;
	NRF_TIMER4->CC[0]		 = 31250;
	NRF_TIMER4->INTENSET	 = 65536;
	NRF_TIMER4->SHORTS		 = 0x01;
	NVIC_EnableIRQ(TIMER4_IRQn);
	NRF_TIMER4->TASKS_START  = 1;

	Serial.println(" >>> Device Initialization Completed");
}

void TIMER4_IRQHandler(void){
	static bool toggle = 0;
	Serial.println("test");
	if(toggle == 0){
		digitalWrite(LED_BT, LOW);
		toggle = 1;
		NRF_TIMER4->EVENTS_COMPARE[0] = 0;
	}else if(toggle == 1){
		digitalWrite(LED_BT, HIGH);
		toggle = 0;
		NRF_TIMER4->EVENTS_COMPARE[0] = 0;
	}
}
