/*
 * baro.c
 *
 *  Created on: Aug 12, 2022
 *      Author: a
 */

#include "baro.h"
#include "main.h"
#include <stm32f4xx_hal.h>
#include <stdbool.h>

extern I2C_HandleTypeDef hi2c1;

static uint16_t dig_T1, dig_P1;
static int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static int32_t t_fine;
static _Bool inited = false;
static uint8_t data[25];

baro_stat_t baro_init(void) {
	HAL_StatusTypeDef status;

	// Check barometer on I2C bus
	status = HAL_I2C_Mem_Read(&hi2c1, BARO_I2C_ADDR, BARO_REG_ID, I2C_MEMADD_SIZE_8BIT, data, 1, 1000);
	if (status != HAL_OK) {
		if (status == HAL_TIMEOUT)
			return BARO_ERR_TIMEOUT;
		else
			return BARO_ERR_IO;
	}

	if (data[0] != BARO_ID_VALUE) {
		return BARO_ERR_GENERAL;
	}

	// Config barometer
	// T and P oversampling - 16, Normal mode
	// SPI 3 wire Off, Filter 2x, Standby duration - 62.5 ms
	data[0] = BARO_MODE_NORMAL | BARO_OSRS_T_1 | BARO_OSRS_P_1;
	data[1] = BARO_SPI3W_OFF | BARO_FILTER_OFF | BARO_T_SB_0_5;

	status = HAL_I2C_Mem_Write(&hi2c1, BARO_I2C_ADDR, BARO_REG_CTRL_MEAS, I2C_MEMADD_SIZE_8BIT, data, 2, 1000);
	if (status != HAL_OK) {
		if (status == HAL_TIMEOUT)
			return BARO_ERR_TIMEOUT;
		else
			return BARO_ERR_IO;
	}

	// Readout compensation values
	status = HAL_I2C_Mem_Read(&hi2c1, BARO_I2C_ADDR, BARO_REG_CALIB00, I2C_MEMADD_SIZE_8BIT, data, 25, 1000);
	if (status != HAL_OK) {
		if (status == HAL_TIMEOUT)
			return BARO_ERR_TIMEOUT;
		else
			return BARO_ERR_IO;
	}
	dig_T1 = (uint16_t)data [1] << 8 | (uint16_t)data [0];
	dig_T2 =  (int16_t)data [3] << 8 |  (int16_t)data [2];
	dig_T3 =  (int16_t)data [5] << 8 |  (int16_t)data [4];
	dig_P1 = (uint16_t)data [7] << 8 | (uint16_t)data [6];
	dig_P2 =  (int16_t)data [9] << 8 |  (int16_t)data [8];
	dig_P3 =  (int16_t)data[11] << 8 |  (int16_t)data[10];
	dig_P4 =  (int16_t)data[13] << 8 |  (int16_t)data[12];
	dig_P5 =  (int16_t)data[15] << 8 |  (int16_t)data[14];
	dig_P6 =  (int16_t)data[17] << 8 |  (int16_t)data[16];
	dig_P7 =  (int16_t)data[19] << 8 |  (int16_t)data[18];
	dig_P8 =  (int16_t)data[21] << 8 |  (int16_t)data[20];
	dig_P9 =  (int16_t)data[23] << 8 |  (int16_t)data[22];

	inited = true;

	return BARO_OK;
}

int32_t baro_read_temp(void) {
	int32_t var1, var2, temp;

	if (!inited)
		return INT32_MAX;

	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(&hi2c1, BARO_I2C_ADDR, BARO_REG_TEMP_MSB, I2C_MEMADD_SIZE_8BIT, data, 3, 1000);
	if (status != HAL_OK) {
		return INT32_MAX;
	}
	int32_t adc_T = data[0] << 12 | data[1] << 4 | data[2] >> 4;

	var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) *
		   ((int32_t)dig_T2)) >> 11;

	var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
			  ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
		      ((int32_t)dig_T3)) >> 14;

	t_fine = var1 + var2;

	temp = (t_fine * 5 + 128) >> 8;

	return temp;
}

uint32_t baro_read_press(void) {
	int64_t var1, var2, p;

	if (!inited)
		return INT32_MAX;

	if (baro_read_temp() == INT32_MAX)
		return UINT32_MAX;

	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(&hi2c1, BARO_I2C_ADDR, BARO_REG_PRESS_MSB, I2C_MEMADD_SIZE_8BIT, data, 3, 1000);
	if (status != HAL_OK) {
		return INT32_MAX;
	}
	int32_t adc_P = data[0] << 12 | data[1] << 4 | data[2] >> 4;

	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
	var2 = var2 + (((int64_t)dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) +
		   ((var1 * (int64_t)dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

	if (var1 == 0) {
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
	return p / 256;

}
