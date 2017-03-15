#include "BME280.h"

BME280::BME280() {
	this->addr = BME280_ADDR_SDO_LOW;
	this->useHighAccuracy = false;
}

BME280::BME280(uint8_t addr) {
	this->addr = addr;
	this->useHighAccuracy = false;
}

bool BME280::init() {
	if (!checkConnection())
		return false;

	if (!setPressOversampling(BME280_OVERSAMPLING_1))
		return false;
	if (!setTempOversampling(BME280_OVERSAMPLING_1))
		return false;
	if (!setHumOversampling(BME280_OVERSAMPLING_1))
		return false;
	if (!setMode(BME280_MODE_NORMAL))
		return false;

	return loadCalibrationData();
}

bool BME280::getHighAccuracy() {
	return useHighAccuracy;
}

void BME280::setHighAccuracy(bool enabled) {
	useHighAccuracy = enabled;
}

uint8_t BME280::getDeviceID() {
	I2Cdev::readByte(addr, BME280_RA_ID, buffer);
	return buffer[0];
}

bool BME280::checkConnection() {
	return getDeviceID() == 0x60;
}

bool BME280::reset() {
	return I2Cdev::writeByte(addr, BME280_RA_RESET, 0xB6);
}

uint16_t BME280::readInt16(int offset) {
	return (((uint16_t)buffer[offset + 1]) << 8) | buffer[offset];
}

uint32_t BME280::readInt32(int offset) {
	uint32_t a = buffer[offset];
	uint32_t b = buffer[offset + 1];
	uint32_t c = buffer[offset + 2];
	uint32_t d = buffer[offset + 3];

	return d << 24 | c << 16 | b << 8 | a;
}

uint8_t BME280::getTempOversampling() {
	I2Cdev::readBits(addr, BME280_RA_CTRL_MEAS, 7, 3, buffer);
	return buffer[0];
}

bool BME280::setTempOversampling(uint8_t mode) {
	return I2Cdev::writeBits(addr, BME280_RA_CTRL_MEAS, 7, 3, mode);
}

uint8_t BME280::getPressOversampling() {
	I2Cdev::readBits(addr, BME280_RA_CTRL_MEAS, 4, 3, buffer);
	return buffer[0];
}

bool BME280::setPressOversampling(uint8_t mode) {
	return I2Cdev::writeBits(addr, BME280_RA_CTRL_MEAS, 4, 3, mode);
}

uint8_t BME280::getHumOversampling() {
	I2Cdev::readBits(addr, BME280_RA_CTRL_HUM, 2, 3, buffer);
	return buffer[0];
}

bool BME280::setHumOversampling(uint8_t mode) {
	if (!I2Cdev::writeBits(addr, BME280_RA_CTRL_HUM, 2, 3, mode))
		return false;

	// Changes to this register only become effective after a write operation to “ctrl_meas”.
	if (!I2Cdev::readByte(addr, BME280_RA_CTRL_MEAS, buffer))
		return false;
	return I2Cdev::writeByte(addr, BME280_RA_CTRL_MEAS, buffer[0]);
}

uint8_t BME280::getMode() {
	I2Cdev::readBits(addr, BME280_RA_CTRL_MEAS, 1, 2, buffer);
	return buffer[0];
}

bool BME280::setMode(uint8_t mode) {
	return I2Cdev::writeBits(addr, BME280_RA_CTRL_MEAS, 1, 2, mode);
}

bool BME280::isMeasuring() {
	I2Cdev::readBits(addr, BME280_RA_STATUS, 3, 1, buffer);
	return buffer[0] != 0;
}

uint8_t BME280::getStandbyTime() {
	I2Cdev::readBits(addr, BME280_RA_CONFIG, 7, 3, buffer);
	return buffer[0];
}

bool BME280::setStandbyTime(uint8_t time) {
	return I2Cdev::writeBits(addr, BME280_RA_CONFIG, 7, 3, time);
}

uint8_t BME280::getFilterCoefficient() {
	I2Cdev::readBits(addr, BME280_RA_CONFIG, 4, 3, buffer);
	return buffer[0];
}

bool BME280::setFilterCoefficient(uint8_t coefficient) {
	return I2Cdev::writeBits(addr, BME280_RA_CONFIG, 4, 3, coefficient);
}

bool BME280::loadCalibrationData() {
	if (!I2Cdev::readBytes(addr, BME280_RA_CALIB00, 26, buffer))
		return false;

	dig_T1 = readInt16(0);
	dig_T2 = readInt16(2);
	dig_T3 = readInt16(4);
	dig_P1 = readInt16(6);
	dig_P2 = readInt16(8);
	dig_P3 = readInt16(10);
	dig_P4 = readInt16(12);
	dig_P5 = readInt16(14);
	dig_P6 = readInt16(16);
	dig_P7 = readInt16(18);
	dig_P8 = readInt16(20);
	dig_P9 = readInt16(22);
	dig_H1 = buffer[25];

	if (!I2Cdev::readBytes(addr, BME280_RA_CALIB26, 7, buffer))
		return false;

	dig_H2 = readInt16(0);
	dig_H3 = buffer[2];
	dig_H4 = ((uint16_t)buffer[3] << 4) | (buffer[4] & 0b00001111);
	dig_H5 = ((uint16_t)buffer[5] << 4) | (buffer[4] >> 4);
	dig_H6 = buffer[6];
	return true;
}

bool BME280::getValues(float* temp, float* press, float* hum) {
	if (!I2Cdev::readBytes(addr, BME280_RA_PRESS_MSB, 8, buffer))
		return false;

	int32_t adcPress = 0;
	int32_t adcTemp = 0;
	int32_t adcHum = 0;

	adcPress |= (uint32_t)buffer[0] << 12;
	adcPress |= (uint32_t)buffer[1] << 4;
	adcPress |= (uint32_t)buffer[2] >> 4;

	adcTemp |= (uint32_t)buffer[3] << 12;
	adcTemp |= (uint32_t)buffer[4] << 4;
	adcTemp |= (uint32_t)buffer[5] >> 4;

	adcHum |= (uint32_t)buffer[6] << 8;
	adcHum |= buffer[7];


	if (useHighAccuracy) {
		*temp = (float)compensate_T_double(adcTemp);
		*press = (float)compensate_P_double(adcPress) / 100.0F;
		*hum = (float)compensate_H_double(adcHum);
	}
	else {
		*temp = compensate_T_int32(adcTemp) * 0.01f;
		*press = compensate_P_int64(adcPress) / (256.0f * 100);
		*hum = compensate_H_int32(adcHum) / 1024.0f;
	}
	return true;
}

// BME Data sheet code

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
BME280_S32_t BME280::compensate_T_int32(BME280_S32_t adc_T)
{
	BME280_S32_t var1, var2, T;
	var1 = ((((adc_T >> 3) -((BME280_S32_t)dig_T1 << 1))) * ((BME280_S32_t)dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) -((BME280_S32_t)dig_T1)) * ((adc_T >> 4) - ((BME280_S32_t)dig_T1))) >> 12) *
		((BME280_S32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
BME280_U32_t BME280::compensate_P_int64(BME280_S32_t adc_P)
{
	BME280_S64_t var1, var2, p;
	var1 = ((BME280_S64_t)t_fine) - 128000;
	var2 = var1 * var1 * (BME280_S64_t)dig_P6;
	var2 = var2 + ((var1*(BME280_S64_t)dig_P5) << 17);
	var2 = var2 + (((BME280_S64_t)dig_P4) << 35);
	var1 = ((var1 * var1 * (BME280_S64_t)dig_P3) >> 8) + ((var1 * (BME280_S64_t)dig_P2) << 12);
	var1 = (((((BME280_S64_t)1) << 47) + var1))*((BME280_S64_t)dig_P1) >> 33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((BME280_S64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((BME280_S64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((BME280_S64_t)dig_P7) << 4);
	return (BME280_U32_t)p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
BME280_U32_t BME280::compensate_H_int32(BME280_S32_t adc_H)
{
	BME280_S32_t v_x1_u32r;
	v_x1_u32r = (t_fine -((BME280_S32_t)76800));
	v_x1_u32r = (((((adc_H << 14) -(((BME280_S32_t)dig_H4) << 20) - (((BME280_S32_t)dig_H5) * v_x1_u32r)) +
		((BME280_S32_t)16384)) >> 15) * (((((((v_x1_u32r * ((BME280_S32_t)dig_H6)) >> 10) * (((v_x1_u32r *
		((BME280_S32_t)dig_H3)) >> 11) + ((BME280_S32_t)32768))) >> 10) + ((BME280_S32_t)2097152)) *
		((BME280_S32_t)dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r -(((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((BME280_S32_t)dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (BME280_U32_t)(v_x1_u32r >> 12);
}

// Returns temperature in DegC, double precision. Output value of “51.23” equals 51.23 DegC.
double BME280::compensate_T_double(BME280_S32_t adc_T)
{
	double var1, var2, T;
	var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
	var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) *
		(((double)adc_T)/131072.0 - ((double) dig_T1)/8192.0)) * ((double)dig_T3);
	t_fine = (BME280_S32_t)(var1 + var2);
	T = (var1 + var2) / 5120.0;
	return T;
}

// Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa
double BME280::compensate_P_double(BME280_S32_t adc_P)
{
	double var1, var2, p;
	var1 = ((double)t_fine/2.0) - 64000.0;
	var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)dig_P5) * 2.0;
	var2 = (var2/4.0)+(((double)dig_P4) * 65536.0);
	var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0)*((double)dig_P1);
	if (var1 == 0.0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576.0 - (double)adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double)dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double)dig_P8) / 32768.0;
	p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;
	return p;
}

// Returns humidity in %rH as as double. Output value of “46.332” represents 46.332 %rH
double BME280::compensate_H_double(BME280_S32_t adc_H)
{
	double var_H;
	var_H = (((double)t_fine) - 76800.0);
	var_H = (adc_H - (((double)dig_H4) * 64.0 + ((double)dig_H5) / 16384.0 * var_H)) *
		(((double)dig_H2) / 65536.0 * (1.0 + ((double)dig_H6) / 67108864.0 * var_H *
		(1.0 + ((double)dig_H3) / 67108864.0 * var_H)));
		var_H = var_H * (1.0 - ((double)dig_H1) * var_H / 524288.0);
	if (var_H > 100.0)
		var_H = 100.0;
	else if (var_H < 0.0)
		var_H = 0.0;
	return var_H;
}