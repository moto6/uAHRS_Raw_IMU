/*
 * I2C(TWI) Library for nRF52
 * Last Updated Date : Oct. 22, 2019
 * Poject Manager : S.W. Jeong
 * E-mail : jswcomkr@naver.com
 */

#include "Wire.h"
#include "arduino.h"

_Wire Wire;

void _Wire::begin(){
	// I2C ����� Ȱ��ȭ ��Ű�� �Լ�
	// �⺻ ��� �ӵ��� 400kHz
	// �⺻ ���� �� : SCL(12��), SDA(11��)
	// ���� ���� Wire.h ���� �ٲ� �� ����

	// I2C ��� ��Ȱ��ȭ
	NRF_TWI0->ENABLE = 0;

	// SCL/SDA�� ����
	NRF_TWI0->PSELSCL = pinSCL;
	NRF_TWI0->PSELSDA = pinSDA;

	// ��� ���ļ� ����
	NRF_TWI0->FREQUENCY = 0x06680000;

	// ���ͷ�Ʈ ����
	NRF_TWI0->INTENCLR = 0x044286;

	// I2C ��� Ȱ��ȭ
	NRF_TWI0->ENABLE = 5;

	// Clear Events
	NRF_TWI0->EVENTS_RXDREADY = 0;
	NRF_TWI0->EVENTS_TXDSENT = 0;
}

uint8_t _Wire::requestFrom(uint8_t address, uint8_t bytes){
	// �����͸� ��û�ϴ� �Լ�
	uint8_t recvLength = 0;
	char ch[20];
	NRF_TWI0->EVENTS_RXDREADY = 0;
	NRF_TWI0->EVENTS_BB = 0;
	NRF_TWI0->ADDRESS = address;
	NRF_TWI0->TASKS_STARTRX = 1;
	for(uint8_t i=0;i<bytes;i++){
		while(!NRF_TWI0->EVENTS_BB){}
		NRF_TWI0->EVENTS_BB = 0;
		if(i == bytes - 1){
			NRF_TWI0->TASKS_STOP = 1;
		}else{
			NRF_TWI0->TASKS_SUSPEND = 1;
		}
		while(!NRF_TWI0->EVENTS_RXDREADY){}
		NRF_TWI0->EVENTS_RXDREADY = 0;
		rxBuffer[i] = NRF_TWI0->RXD;
		if(i < bytes - 1){
			while(!NRF_TWI0->EVENTS_SUSPENDED){}
			NRF_TWI0->EVENTS_SUSPENDED = 0;
			NRF_TWI0->TASKS_RESUME = 1;
		}else{
			while(!NRF_TWI0->EVENTS_STOPPED){}

		}
	}

	return recvLength;
}

void _Wire::beginTransmission(uint8_t address){
	// TWI ������ ������ �����ϴ� �Լ�
	NRF_TWI0->ENABLE = 5;
	NRF_TWI0->ADDRESS = address;
	NRF_TWI0->TASKS_STARTTX = 1;
	memset(&rCount, 0x00, sizeof(rCount));
	memset(&rxBuffer, 0x00, sizeof(rxBuffer));
	rCount = 0;
}

uint8_t _Wire::endTransmission(bool toggle){
	uint8_t err_code = 0;
	if(toggle){
		if(!NRF_TWI0->EVENTS_STOPPED){NRF_TWI0->TASKS_STOP = 1;}
		while(!NRF_TWI0->EVENTS_STOPPED){}
		NRF_TWI0->EVENTS_STOPPED = 0;
		err_code = 1;
	}
	NRF_TWI0->ENABLE = 0;
	return err_code;
}

uint8_t _Wire::write(uint8_t data){
	NRF_TWI0->TXD = data;
	while(!NRF_TWI0->EVENTS_TXDSENT){}
	NRF_TWI0->EVENTS_TXDSENT = 0;
	return 0;
}

uint8_t _Wire::available(){
	return 0;
}

uint8_t _Wire::read(){
	static uint8_t output = 0;
	output = rxBuffer[rCount];
	rCount++;
	return output;
}

void _Wire::setClock(uint32_t frequency){
	// I2C ��� �ӵ��� �����ϴ� �Լ�
	// ������ �Լ��� ȣ������ ���� ��� 400kHz�� ����
	switch(frequency){
	case 100000:
		// 100Khz�� ���� ��
		NRF_TWI1->FREQUENCY = 0x01980000;
		break;
	case 250000:
		// 250kHz�� ���� ��
		NRF_TWI1->FREQUENCY = 0x04000000;
		break;
	case 400000:
		// 400kHz�� ���� ��
		NRF_TWI1->FREQUENCY = 0x06680000;
		break;
	default:
		NRF_TWI1->FREQUENCY = 0x06680000;
		break;
	}
}
