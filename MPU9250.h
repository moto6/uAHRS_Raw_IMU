/*
 * Header for MPU9250 IMU Sensor
 * Creator : Seonguk Jeong
 * Last Updated : Nov. 11, 2019
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#include "stdio.h"
#include "Wire.h"
#include "Arduino.h"
#include "math.h"
#include "nrf_nvmc.h"
#include "nrf_sdh.h"

#define MPU9250 0x68

// Define Register
#define XG_OFFSET_H 0x13
#define XG_OFFSET_L 0x14
#define YG_OFFSET_H 0x15
#define YG_OFFSET_L 0x16
#define ZG_OFFSET_H 0x17
#define ZG_OFFSET_L 0x18
#define SMPLRT_DIV 0x19
#define _CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D
#define LP_ACCEL_ODR 0x1E
#define WOM_THR 0x1F
#define FIFO_EN 0x23
#define I2C_MST_CTRL 0x24
#define I2C_SLV0_ADDR 0x25
#define I2C_SLV0_REG 0x26
#define I2C_SLV0_CTRL 0x27
#define I2C_SLV1_ADDR 0x28
#define I2C_SLV1_REG 0x29
#define I2C_SLV1_CTRL 0x2A
#define I2C_SLV2_ADDR 0x2B
#define I2C_SLV2_REG 0x2C
#define I2C_SLV2_CTRL 0x2D
#define I2C_SLV3_ADDR 0x2E
#define I2C_SLV3_REG 0x2F
#define I2C_SLV3_CTRL 0x30
#define I2C_SLV4_ADDR 0x31
#define I2C_SLV4_REG 0x32
#define I2C_SLV4_CTRL 0x33
#define I2C_SLV4_DI 0x35
#define I2C_MST_STATUS 0x36
#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38
#define INT_STATUS 0x3A
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define FIFO_COUNTH 0x72
#define FIFO_COUNTL 0x73
#define FIFO_R_W 0x74
#define WHO_AM_I 0x75
#define XA_OFFSET_H 0x77
#define XA_OFFSET_L 0x78
#define YA_OFFSET_H 0x7A
#define YA_OFFSET_L 0x7B
#define ZA_OFFSET_H 0x7D
#define ZA_OFFSET_L 0x7E

// Define AK8963 Register
#define AK8963 0x0C
#define WIA 0x00
#define INFO 0x01
#define ST1 0x02
#define HXL 0x03
#define HXH 0x04
#define HYL 0x05
#define HYH 0x06
#define HZL 0x07
#define HZH 0x08
#define ST2 0x09
#define CNTL1 0x0A
#define CNTL2 0x0B
#define ASTC 0x0C
#define TS1 0x0D
#define TS2 0x0E
#define I2CDIS 0x0F
#define ASAX 0x10
#define ASAY 0x11
#define ASAZ 0x12

// Pre-defined Settings
#define GYRO_SET_250DPS 0x00
#define GYRO_SET_500DPS 0x08
#define GYRO_SET_1000DPS 0x10
#define GYRO_SET_2000DPS 0x18
#define GYRO_DLPF_EN 0x00
#define GYRO_DLPF_DIS 0x01
#define GYRO_LSB_250DPS 131.0f
#define GYRO_LSB_500DPS 65.5f
#define GYRO_LSB_1000DPS 32.8f
#define GYRO_LSB_2000DPS 16.4f

#define ACCEL_SET_2G 0x00
#define ACCEL_SET_4G 0x08
#define ACCEL_SET_8G 0x10
#define ACCEL_SET_16G 0x18
#define ACCEL_DLPF_EN 0x00
#define ACCEL_DLPF_DIS 0x04


#define DLPF_CFG_0 0x00
#define DLPF_CFG_1 0x01
#define DLPF_CFG_2 0x02
#define DLPF_CFG_3 0x03
#define DLPF_CFG_4 0x04
#define DLPF_CFG_5 0x05
#define DLPF_CFG_6 0x06
#define DLPF_CFG_7 0x07

#define LPS25 0x5C
#define REF_P_XL 0x08
#define REP_P_L 0x09
#define REP_P_H 0x0A
#define WHOAMI 0x0F
#define RES_CONF 0x10
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define INTERRUPT_CFG 0x24
#define INT_SOURCE 0x25
#define STATUS_REG 0x27
#define PRESS_OUT_XL 0x28
#define PRESS_OUT_L 0x29
#define PRESS_OUT_H 0x2A
#define FIFO_CTRL 0x2E
#define FIFO_STATUS 0x2F
#define THS_P_L 0x30
#define THS_P_H 0x31
#define RPDS_L 0x39
#define RPDS_H 0x3A

class _MPU9250{
private:
	int16_t ASA[3] = {0}, offsetHard[3] = {0}, offsetSoft[3] = {0};
	void writeData(uint8_t deviceId, uint8_t reg_addr, uint8_t data);
	int16_t *getOffsetAccel();
	int16_t *getOffsetGyro();
public:
	void Init();
	bool getDeviceId();
	bool getMagId();
	int16_t *getRawAccel();
	int16_t *getRawGyro();
	int16_t *getRawMag();
	int16_t *getCalibratedMag();
	void caliAccel();
	void caliGyro();
	void caliMag();
};

#endif /* MPU9250_H_ */
