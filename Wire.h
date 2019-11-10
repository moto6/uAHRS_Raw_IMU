/*
 * I2C(TWI) Library for nRF52
 * Last Updated Date : Oct. 22, 2019
 * Poject Manager : S.W. Jeong
 * E-mail : jswcomkr@naver.com
 */

#ifndef WIRE_H_
#define WIRE_H_

#include <nrf_twi.h>
#include <stdint.h>

#define pinSDA 11
#define pinSCL 12

class _Wire{
public:
	void begin();
	uint8_t rxBuffer[256] = {0}, rCount = 0;
	uint8_t requestFrom(uint8_t address, uint8_t bytes);
	void beginTransmission(uint8_t address);
	uint8_t endTransmission(bool toggle);
	uint8_t write(uint8_t data);
	uint8_t available();
	uint8_t read();
	void setClock(uint32_t frequency);
};

#endif
