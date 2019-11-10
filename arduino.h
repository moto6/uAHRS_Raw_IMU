/*
 * nRduino Project Library
 *
 * Current version : v0.00.01
 * Last update date : Oct. 21, 2019
 * Poject Manager : S.W. Jeong
 * E-mail : jswcomkr@naver.com
 *
 */
#ifndef ARDUINO_H_
#define ARDUINO_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <nrf_rtc.h>
#include <nrf_clock.h>
#include <nrf_saadc.h>
#include <nrf_uart.h>
#include <nrf_gpio.h>
#include <nrf_delay.h>

// 기본 매크로
#define OUTPUT 1
#define INPUT 0
#define HIGH 1
#define LOW 0
#define PI 3.141592f


// ADC 관련 매크로
#define A0 NRF_SAADC_INPUT_AIN0
#define A1 NRF_SAADC_INPUT_AIN1
#define A2 NRF_SAADC_INPUT_AIN2
#define A3 NRF_SAADC_INPUT_AIN3
#define A4 NRF_SAADC_INPUT_AIN4
#define A5 NRF_SAADC_INPUT_AIN5
#define A6 NRF_SAADC_INPUT_AIN6
#define A7 NRF_SAADC_INPUT_AIN7

// USB to UART 관련 매크로
#define UART_RX_PIN 8
#define UART_TX_PIN 6

// Arduino 함수 Pre-define
int16_t map(int16_t val, int32_t _min, int32_t _max, int32_t min, int32_t max);
int32_t constrain(int32_t val, int32_t min, int32_t max);
void arduinoInit();
class _Serial{
public:
	void begin(uint32_t buadrate);
	void end();
	void print(const char *texts);
	void println(const char *texts);
	uint8_t read();
};
void delay(uint16_t ms);
void rtc_init();
void saadc_init();
uint32_t millis();
uint16_t analogRead(nrf_saadc_input_t portNo);
void analogWrite();
void pinMode(uint8_t port, bool dir);
void digitalWrite(uint8_t port, bool value);
#endif
