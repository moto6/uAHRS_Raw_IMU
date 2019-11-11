/*
 * Sources for MPU9250 IMU Sensor
 * Creator : Seonguk Jeong
 * Last Updated : Nov. 11, 2019
 */

#include "MPU9250.h"

extern _Wire Wire;
extern _Serial Serial;

void _MPU9250::Init(){
	// TWI ����
	Wire.begin();
	Serial.println(" >>> Initialize IMU");

	// IMU ���� ����
	writeData(MPU9250, PWR_MGMT_1, 0x80);
	delay(100);
	writeData(MPU9250, PWR_MGMT_1, 0x00);

	// Bypass ��� Ȱ��ȭ
	writeData(MPU9250, INT_PIN_CFG, 0X02);

	// ���ڱ⼾�� ����
	writeData(AK8963, CNTL2, 0x01);
	delay(10);
	writeData(AK8963, CNTL2, 0x00);

	// LPF ����
	// DLPF 10Hz
	writeData(MPU9250, _CONFIG, 0X05);

	// ���̷ν����� ����
	// Default : 1000DPS
	writeData(MPU9250, GYRO_CONFIG, 0x10);

	// ���ӵ� ���� �������� ����
	// Default : 2g
	writeData(MPU9250, ACCEL_CONFIG, 0x00);

	// ���ӵ� ���� DLPF����
	// Default : 10Hz
	writeData(MPU9250, ACCEL_CONFIG2, 0x05);

	// ���ڱ⼾�� ��� ����
	// Fuse ROM Access ���
	writeData(AK8963, CNTL1, 0x00);
	delay(100);
	writeData(AK8963, CNTL1, 0x0f);
	delay(100);

	// ASA �� ����
	Wire.beginTransmission(AK8963);
	Wire.write(ASAX);
	Wire.requestFrom(AK8963, 3);
	for(uint8_t i=0;i<3;i++){
		ASA[i] = Wire.read();
	}
	Wire.endTransmission(true);

	// Continous Mode 2�� ����(16-BIT)
	writeData(AK8963, CNTL1, 0x00);
	delay(100);
	writeData(AK8963, CNTL1, 0x16);
	delay(100);

	// MPU9250 ID �о��
	if(getDeviceId()){Serial.println(" >>> IMU Ready");}
	else{Serial.println(" >>> IMU Failed! [ERROR]");}
	if(getMagId()){Serial.println(" >>> MAG Ready");}
	else{Serial.println(" >>> MAG Failed! [ERROR]");}

	// UICR �������Ϳ� ����� �����°��� ������ ���ε�
	for(uint8_t i=0;i<3;i++){
		writeData(MPU9250, XG_OFFSET_H + (2*i), NRF_UICR->CUSTOMER[i] >> 8 & 0xff);
		writeData(MPU9250, XG_OFFSET_L + (2*i), NRF_UICR->CUSTOMER[i]  & 0xff);
		writeData(MPU9250, XA_OFFSET_H + (3*i), NRF_UICR->CUSTOMER[i+3] >> 8 & 0xff);
		writeData(MPU9250, XA_OFFSET_L + (3*i), NRF_UICR->CUSTOMER[i+3]  & 0xff);
		offsetHard[i] = NRF_UICR->CUSTOMER[i+6];
		offsetSoft[i] = NRF_UICR->CUSTOMER[i+9];
	}
	char ch[40];
	sprintf(ch, "%d %d %d", offsetHard[0],  offsetHard[1],  offsetHard[2]);
	Serial.println(ch);
}
bool _MPU9250::getDeviceId(){
	uint8_t output = 0;
	Wire.beginTransmission(MPU9250);
	Wire.write(WHO_AM_I);
	Wire.requestFrom(MPU9250, 1);
	output = Wire.read();
	Wire.endTransmission(true);
	if(output == 113){return 1;}
	else{return 0;}
}
bool _MPU9250::getMagId(){
	uint8_t output = 0;
	Wire.beginTransmission(AK8963);
	Wire.write(WIA);
	Wire.requestFrom(AK8963, 1);
	output = Wire.read();
	Wire.endTransmission(true);
	if(output == 72){return 1;}
	else{return 0;}
}
int16_t* _MPU9250::getRawAccel(){
	static int16_t output[3] = {0};
	Wire.beginTransmission(MPU9250);
	Wire.write(ACCEL_XOUT_H);
	Wire.requestFrom(MPU9250, 6);
	for(uint8_t i=0;i<3;i++){
		output[i] = Wire.read() << 8 | Wire.read();
	}
	Wire.endTransmission(true);
	return output;
}
int16_t* _MPU9250::getRawGyro(){
	static int16_t output[3] = {0};
	Wire.beginTransmission(MPU9250);
	Wire.write(GYRO_XOUT_H);
	Wire.requestFrom(MPU9250, 6);
	for(uint8_t i=0;i<3;i++){
		output[i] = Wire.read() << 8 | Wire.read();
	}
	Wire.endTransmission(true);
	return output;
}
int16_t* _MPU9250::getRawMag(){
	static int16_t output[4] = {0};

	// Read ST1 Register
	Wire.beginTransmission(AK8963);
	Wire.write(ST1);
	Wire.requestFrom(AK8963, 1);
	uint8_t st1 = Wire.read();
	Wire.endTransmission(true);

	if(st1 & 0x01){
		// Read Magnetometer
		Wire.beginTransmission(AK8963);
		Wire.write(HXL);
		Wire.requestFrom(AK8963, 7);
		for(uint8_t i=0;i<3;i++){
			output[i] = Wire.read();
			output[i] = (int8_t)Wire.read() << 8 | output[i];
		}
		Wire.endTransmission(true);

		// ASA �������� �����͸� �����Ѵ�.
		for(uint8_t i=0;i<3;i++){
			output[i] = (float)output[i] * ((((float)ASA[i]-128.0f)*0.5f/ 128.0f) + 1.0f);
			output[i] = (float)output[i]*0.15f;		// uT ������ ��ȯ
		}
		output[3] = 1;
		return output;
	}else{
		memset(&output, 0, sizeof(output));
		return output;
	}
}
int16_t* _MPU9250::getCalibratedMag(){
	static int16_t output[4] = {0};
	int16_t *MAG;
	MAG = getRawMag();
	for(uint8_t i=0;i<3;i++){
		output[i] = MAG[i] - offsetHard[i];
	}
	output[3] = MAG[3];
	return output;
}
void _MPU9250::caliAccel(){
	// ���ӵ� ���� ���� �˰���
	int16_t *ACCEL, *offAccel, itinary = 3000;

	do{
		int32_t sum[3] = {0}, avg[3] = {0}, count = 0, offset[3] = {0};
		offAccel = getOffsetAccel();

		// ���̷� ���� ���� ����Ѵ�
		do{
			ACCEL = getRawAccel();
			for(uint8_t i=0;i<3;i++){
				sum[i] += ACCEL[i];
			}
			count++;
		}while(count < itinary);

		// ���� ����� ����Ѵ�
		// ����� ��� ���� ������ �������Ϳ� ����
		for(uint8_t i=0;i<3;i++){
			offset[i] = offAccel[i] - (sum[i] / 8 / itinary);
			if(i==2){offset[i] += 2048;}
			writeData(MPU9250, XA_OFFSET_H + (i*3), (offset[i] >> 8) & 0xff);
			writeData(MPU9250, XA_OFFSET_L + (i*3), offset[i] & 0xff);
		}

		// �ٽ� ��հ��� ����Ѵ�
		count = 0;
		memset(&sum, 0, sizeof(sum));
		memset(&avg, 0, sizeof(avg));
		do{
			ACCEL = getRawAccel();
			for(uint8_t i=0;i<3;i++){
				sum[i] += ACCEL[i];
			}
			count++;
		}while(count < itinary);

		for(uint8_t i=0;i<3;i++){
			avg[i] = sum[i] / itinary;
		}
		avg[2] -= 16384;

		// ����� ���� �Ǵ��Ѵ�
		int32_t check = sqrt(avg[0]*avg[0] + avg[1]*avg[1] + avg[2]*avg[2]);

		// ����� ���� �Ǵ��Ѵ�
		if(check <= 50){
			Serial.println(" >>> Accelerometer Calibration Completed!");

			// Softdevice ��Ȱ��ȭ
			//while(nrf_sdh_disable_request()){}

			// UICR �������� ���
			int32_t backup[12] = {0};
			NRF_NVMC->CONFIG = 0x00;

			// ���̷ν����� ���尪 ���
			for(uint8_t i=0;i<3;i++){
				backup[i] = NRF_UICR->CUSTOMER[i];
			}

			// ���ڱ� ���� Hard Iron ���尪 ���
			for(uint8_t i=6;i<9;i++){
				backup[i] = NRF_UICR->CUSTOMER[i];
			}

			// ���ڱ� ���� Soft Iron ���尪 ���
			for(uint8_t i=9;i<12;i++){
				backup[i] = NRF_UICR->CUSTOMER[i];
			}

			// UICR �������� �ʱ�ȭ
			NRF_NVMC->CONFIG = 0x02;
			NRF_NVMC->ERASEUICR = 0x01;
			while(NRF_NVMC->READY == NVMC_READY_READY_Busy){}

			// UICR �������Ϳ� ������ �Է�
			for(uint8_t i=0;i<3;i++){
				NRF_NVMC->CONFIG = 0x01;
				NRF_UICR->CUSTOMER[i] = backup[i];
				while(NRF_NVMC->READY == NVMC_READY_READY_Busy){}
				NRF_UICR->CUSTOMER[i+3] = offset[i];
				while(NRF_NVMC->READY == NVMC_READY_READY_Busy){}
				NRF_UICR->CUSTOMER[i+6] = backup[i+6];
				while(NRF_NVMC->READY == NVMC_READY_READY_Busy){}
				NRF_UICR->CUSTOMER[i+9] = backup[i+9];
				while(NRF_NVMC->READY == NVMC_READY_READY_Busy){}
			}
			NRF_NVMC->CONFIG = 0x01;

			break;
		}
	}while(1);
}
void _MPU9250::caliGyro(){
	// ���̷ν����� ���� �˰���
	int16_t *GYRO, *offGyro, itinary = 3000;

	// ���̷ν����� ������ ����
	for(uint8_t i=0;i<3;i++){
		writeData(MPU9250, XG_OFFSET_H + (i*2), 0);
		writeData(MPU9250, XG_OFFSET_L + (i*2), 0);
	}

	do{
		int32_t sum[3] = {0}, avg[3] = {0}, count = 0, offset[3] = {0};
		offGyro = getOffsetGyro();

		// ���̷� ���� ���� ����Ѵ�
		do{
			GYRO = getRawGyro();
			for(uint8_t i=0;i<3;i++){
				sum[i] += GYRO[i];
			}
			count++;
		}while(count < itinary);

		// ���� ����� ����Ѵ�
		// ����� ��� ���� ������ �������Ϳ� ����
		for(uint8_t i=0;i<3;i++){
			offset[i] = offGyro[i] - sum[i] / itinary;
			writeData(MPU9250, XG_OFFSET_H + (i*2), (offset[i] >> 8) & 0xff);
			writeData(MPU9250, XG_OFFSET_L + (i*2), offset[i] & 0xff);
		}

		// �ٽ� ��հ��� ����Ѵ�
		count = 0;
		memset(&sum, 0, sizeof(sum));
		memset(&avg, 0, sizeof(avg));
		do{
			GYRO = getRawGyro();
			for(uint8_t i=0;i<3;i++){
				sum[i] += GYRO[i];
			}
			count++;
		}while(count < itinary);

		for(uint8_t i=0;i<3;i++){
			avg[i] = sum[i] / itinary;
		}

		// ����� ���� �Ǵ��Ѵ�
		int32_t check = sqrt(avg[0]*avg[0] + avg[1]*avg[1] + avg[2]*avg[2]);

		if(check <= 10){
			Serial.println(" >>> Gyroscope Calibration Completed!");

			// Softdevice ��Ȱ��ȭ
			//while(nrf_sdh_disable_request()){}

			// UICR �������� ���
			int32_t backup[12] = {0};
			NRF_NVMC->CONFIG = 0x00;

			// ���ӵ� ���尪 ���
			for(uint8_t i=3;i<6;i++){
				backup[i] = NRF_UICR->CUSTOMER[i];
			}

			// ���ڱ� ���� Hard Iron ���尪 ���
			for(uint8_t i=6;i<9;i++){
				backup[i] = NRF_UICR->CUSTOMER[i];
			}

			// ���ڱ� ���� Soft Iron ���尪 ���
			for(uint8_t i=9;i<12;i++){
				backup[i] = NRF_UICR->CUSTOMER[i];
			}

			// UICR �������� �ʱ�ȭ
			NRF_NVMC->CONFIG = 0x02;
			NRF_NVMC->ERASEUICR = 0x01;
			while(NRF_NVMC->READY == NVMC_READY_READY_Busy){}

			// UICR �������Ϳ� ������ �Է�
			for(uint8_t i=0;i<3;i++){
				NRF_NVMC->CONFIG = 0x01;
				NRF_UICR->CUSTOMER[i] = offset[i];
				while(NRF_NVMC->READY == NVMC_READY_READY_Busy){}
				NRF_UICR->CUSTOMER[i+3] = backup[i+3];
				while(NRF_NVMC->READY == NVMC_READY_READY_Busy){}
				NRF_UICR->CUSTOMER[i+6] = backup[i+6];
				while(NRF_NVMC->READY == NVMC_READY_READY_Busy){}
				NRF_UICR->CUSTOMER[i+9] = backup[i+9];
				while(NRF_NVMC->READY == NVMC_READY_READY_Busy){}
			}
			NRF_NVMC->CONFIG = 0x01;

			break;
		}
	}while(1);
}
void _MPU9250::caliMag(){
	// ���ڱ� ���� ���� �˰���
	// Hard Iron Compensation
	int16_t *MAG, MAX[3] = {0}, MIN[3] = {0}, count = 0, itinary = 2000;
	int32_t  timeNow = millis(), avg[3] = {0};

	// ��հ� ���
	do{
		MAG = getRawMag();
		for(uint8_t i=0;i<3;i++){
			avg[i] += MAG[i];
		}
		count++;
	}while(count < itinary);
	for(uint8_t i=0;i<3;i++){
		avg[i] /= itinary;
		MAX[i] = avg[i];
		MIN[i] = avg[i];
	}

	// Offset ���
	do{
		MAG = getRawMag();
		for(uint8_t i=0;i<3;i++){
			if(MAG[i] >= MAX[i]){MAX[i] = MAG[i];}
			else if(MAG[i] <= MIN[i]){MIN[i] = MAG[i];}
		}
	}while(millis() <= (timeNow+5000));
	for(uint8_t i=0;i<3;i++){
		offsetHard[i] = (MAX[i] + MIN[i]) / 2;
	}

	char ch[40];
	sprintf(ch, "%d %d %d", offsetHard[0], offsetHard[1], offsetHard[2]);
	Serial.println(ch);

	// Soft Iron Compensation

	// UICR�������Ϳ� ���� �����Ѵ�
	// Softdevice ��Ȱ��ȭ
	//while(nrf_sdh_disable_request()){}

	// UICR �������� ���
	int32_t backup[6] = {0};
	NRF_NVMC->CONFIG = 0x00;

	// ���̷� ���尪 ���
	for(uint8_t i=0;i<3;i++){
		backup[i] = NRF_UICR->CUSTOMER[i];
	}

	// ���ӵ� ���尪 ���
	for(uint8_t i=3;i<6;i++){
		backup[i] = NRF_UICR->CUSTOMER[i];
	}

	// UICR �������� �ʱ�ȭ
	NRF_NVMC->CONFIG = 0x02;
	NRF_NVMC->ERASEUICR = 0x01;
	while(NRF_NVMC->READY == NVMC_READY_READY_Busy){}

	// UICR �������Ϳ� ������ �Է�
	for(uint8_t i=0;i<3;i++){
		NRF_NVMC->CONFIG = 0x01;
		NRF_UICR->CUSTOMER[i] = backup[i];
		while(NRF_NVMC->READY == NVMC_READY_READY_Busy){}
		NRF_UICR->CUSTOMER[i+3] = backup[i+3];
		while(NRF_NVMC->READY == NVMC_READY_READY_Busy){}
		NRF_UICR->CUSTOMER[i+6] = offsetHard[i];
		while(NRF_NVMC->READY == NVMC_READY_READY_Busy){}
		NRF_UICR->CUSTOMER[i+9] = offsetSoft[i];
		while(NRF_NVMC->READY == NVMC_READY_READY_Busy){}
	}
	NRF_NVMC->CONFIG = 0x01;
}

int16_t* _MPU9250::getOffsetAccel(){
	static int16_t output[3] = {0};
	Wire.beginTransmission(MPU9250);
	Wire.write(XA_OFFSET_H);
	Wire.requestFrom(MPU9250, 9);
	for(uint8_t i=0;i<3;i++){
		output[i] = Wire.read() << 8 | Wire.read();
		Wire.read();
	}
	Wire.endTransmission(true);
	return output;
}

int16_t* _MPU9250::getOffsetGyro(){
	static int16_t output[3] = {0};
	Wire.beginTransmission(MPU9250);
	Wire.write(XG_OFFSET_H);
	Wire.requestFrom(MPU9250, 6);
	for(uint8_t i=0;i<3;i++){
		output[i] = Wire.read() << 8 | Wire.read();
	}
	Wire.endTransmission(true);
	return output;
}

void _MPU9250::writeData(uint8_t deviceId, uint8_t reg_addr, uint8_t data){
	// I2C�� ������ ����
	Wire.beginTransmission(deviceId);
	Wire.write(reg_addr);
	Wire.write(data);
	Wire.endTransmission(true);
}
