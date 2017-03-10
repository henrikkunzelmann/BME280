// BME280 I2C device class
// Based on "BME280: Final data sheet" Revision 1.2 (October 26th, 2015) [BST-BME280-DS001-11]
// 2016-2017 Henrik Kunzelmann

// May be used with BMP280 sensor, see data sheet for changes

#ifndef _BME280_h
#define _BME280_h

#include "arduino.h"
#include "I2Cdev.h"

#define BME280_ADDR_SDO_LOW 		  0x76
#define BME280_ADDR_SDO_HIGH 		  0x77

#define BME280_RA_HUM_LSB			  0xFE
#define BME280_RA_HUM_MSB			  0xFD

#define BME280_RA_TEMP_XLSB			  0xFC
#define BME280_RA_TEMP_LSB			  0xFB
#define BME280_RA_TEMP_MSB			  0xFA

#define BME280_RA_PRESS_XLSB		  0xF9
#define BME280_RA_PRESS_LSB			  0xF8
#define BME280_RA_PRESS_MSB			  0xF7

#define BME280_RA_CONFIG			  0xF5
#define BME280_RA_CTRL_MEAS			  0xF4
#define BME280_RA_STATUS			  0xF3
#define BME280_RA_CTRL_HUM			  0xF2

#define BME280_RA_CALIB26			  0xE1

#define BME280_RA_RESET				  0xE0
#define BME280_RA_ID				  0xD0
#define BME280_RA_CALIB00			  0x88

#define BME280_OVERSAMPLING_SKIPPED	  0b000
#define BME280_OVERSAMPLING_1		  0b001
#define BME280_OVERSAMPLING_2		  0b010
#define BME280_OVERSAMPLING_4		  0b011
#define BME280_OVERSAMPLING_8		  0b100
#define BME280_OVERSAMPLING_16		  0b101

#define BME280_MODE_SLEEP		      0b00
#define BME280_MODE_FORCED            0b10
#define BME280_MODE_NORMAL            0b11

#define BME280_STANDBY_0_5MS		  0b000
#define BME280_STANDBY_62_5MS		  0b001
#define BME280_STANDBY_125MS		  0b010
#define BME280_STANDBY_250MS		  0b011
#define BME280_STANDBY_500MS		  0b100
#define BME280_STANDBY_1000MS		  0b101
#define BME280_STANDBY_10MS			  0b110
#define BME280_STANDBY_20MS		      0b111

#define BME280_COEFFICIENT_OFF		  0b000
#define BME280_COEFFICIENT_2		  0b000
#define BME280_COEFFICIENT_4		  0b000
#define BME280_COEFFICIENT_8		  0b000
#define BME280_COEFFICIENT_16		  0b000

#define BME280_S32_t				  int32_t
#define BME280_U32_t				  uint32_t
#define BME280_S64_t				  int64_t

class BME280 {
private:
	uint8_t addr;
	uint8_t buffer[26];

	// BME code
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;

	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;

	uint8_t  dig_H1;
	int16_t  dig_H2;
	uint8_t  dig_H3;
	int16_t  dig_H4;
	int16_t  dig_H5;
	int8_t   dig_H6;

	BME280_S32_t t_fine;

	// BME280 datasheet provides more accuracy with double code
	BME280_S32_t compensate_T_int32(BME280_S32_t adc_T);
	BME280_U32_t compensate_P_int64(BME280_S32_t adc_P);
	BME280_U32_t compensate_H_int32(BME280_S32_t adc_H);

public:
	explicit BME280();
	explicit BME280(uint8_t addr);

	bool init();

	uint8_t getDeviceID();
	bool checkConnection();
	bool reset();

	uint16_t readInt16(int offset);
	uint32_t readInt32(int offset);

	uint8_t getTempOversampling();
	bool setTempOversampling(uint8_t mode);

	uint8_t getPressOversampling();
	bool setPressOversampling(uint8_t mode);

	uint8_t getHumOversampling();
	bool setHumOversampling(uint8_t mode);

	uint8_t getMode();
	bool setMode(uint8_t mode);

	uint8_t getStandbyTime();
	bool setStandbyTime(uint8_t time);

	uint8_t getFilterCoefficient();
	bool setFilterCoefficient(uint8_t coefficient);

	bool loadCalibrationData();

	// temp in °C, press in hPa, hum in %RH
	bool getValues(float* temp, float* press, float* hum);
};

#endif