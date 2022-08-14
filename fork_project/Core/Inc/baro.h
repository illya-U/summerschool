/*
 * baro.h
 *
 * Library for communication with BMP280 barometer
 *
 *  Created on: Aug 12, 2022
 *      Author: a
 */

#ifndef INC_BARO_H_
#define INC_BARO_H_

#include "main.h"
#include <stm32f4xx_hal.h>

/**
 * @brief	BMP280 I2C Address
 */
#define BARO_I2C_ADDR		(0x76 << 1)

/**
 * @brief	BMP280 Registers
 */
#define BARO_REG_ID			0xD0
#define BARO_REG_RESET		0xE0
#define BARO_REG_STATUS		0xF3
#define BARO_REG_CTRL_MEAS	0xF4
#define BARO_REG_CONFIG		0xF5
#define BARO_REG_PRESS_MSB	0xF7
#define BARO_REG_PRESS_LSB	0xF8
#define BARO_REG_PRESS_XLSB	0xF9
#define BARO_REG_TEMP_MSB	0xFA
#define BARO_REG_TEMP_LSB	0xFB
#define BARO_REG_TEMP_XLSB	0xFC
#define BARO_REG_CALIB00	0x88
#define BARO_REG_CALIB01	0x89
#define BARO_REG_CALIB02	0x8A
#define BARO_REG_CALIB03	0x8B
#define BARO_REG_CALIB04	0x8C
#define BARO_REG_CALIB05	0x8D
#define BARO_REG_CALIB06	0x8E
#define BARO_REG_CALIB07	0x8F
#define BARO_REG_CALIB08	0x90
#define BARO_REG_CALIB09	0x91
#define BARO_REG_CALIB10	0x92
#define BARO_REG_CALIB11	0x93
#define BARO_REG_CALIB12	0x94
#define BARO_REG_CALIB13	0x95
#define BARO_REG_CALIB14	0x96
#define BARO_REG_CALIB15	0x97
#define BARO_REG_CALIB16	0x98
#define BARO_REG_CALIB17	0x99
#define BARO_REG_CALIB18	0x9A
#define BARO_REG_CALIB19	0x9B
#define BARO_REG_CALIB20	0x9C
#define BARO_REG_CALIB21	0x9D
#define BARO_REG_CALIB22	0x9E
#define BARO_REG_CALIB23	0x9F
#define BARO_REG_CALIB24	0xA0
#define BARO_REG_CALIB25	0xA1

/**
 * @brief	BMP280 ID
 */
#define BARO_ID_VALUE		0x58

/**
 * @brief	BMP280 different registers configs
 */
#define BARO_MODE_SLEEP		(0x0 << 0)
#define BARO_MODE_FORCED	(0x1 << 0)
#define BARO_MODE_NORMAL	(0x3 << 0)
#define BARO_OSRS_T_SKIP	(0x0 << 2)
#define	BARO_OSRS_T_1		(0x1 << 2)
#define	BARO_OSRS_T_2		(0x2 << 2)
#define	BARO_OSRS_T_4		(0x3 << 2)
#define	BARO_OSRS_T_8		(0x4 << 2)
#define	BARO_OSRS_T_16		(0x5 << 2)
#define BARO_OSRS_P_SKIP	(0x0 << 5)
#define	BARO_OSRS_P_1		(0x1 << 5)
#define	BARO_OSRS_P_2		(0x2 << 5)
#define	BARO_OSRS_P_4		(0x3 << 5)
#define	BARO_OSRS_P_8		(0x4 << 5)
#define	BARO_OSRS_P_16		(0x5 << 5)
#define BARO_SPI3W_OFF		(0x0 << 0)
#define BARO_SPI3W_ON		(0x1 << 0)
#define BARO_FILTER_OFF		(0x0 << 1)
#define BARO_FILTER_2		(0x02 << 1)
#define BARO_FILTER_4		(0x04 << 1)
#define BARO_FILTER_8		(0x08 << 1)
#define BARO_FILTER_16		(0x10 << 1)
#define BARO_T_SB_0_5		(0x0 << 5)
#define BARO_T_SB_62_5		(0x1 << 5)
#define BARO_T_SB_125		(0x2 << 5)
#define BARO_T_SB_250		(0x3 << 5)
#define BARO_T_SB_500		(0x4 << 5)
#define BARO_T_SB_1000		(0x5 << 5)
#define BARO_T_SB_2000		(0x6 << 5)
#define BARO_T_SB_4000		(0x7 << 5)

/**
 * @brief	Return state type
 */
typedef enum {
	BARO_OK,
	BARO_ERR_IO,
	BARO_ERR_TIMEOUT,
	BARO_ERR_GENERAL
} baro_stat_t;

/**
 *	@name 	baro_init
 *	@brief	Initialise barometer
 *	@param	none
 *	@return	Return state
 */
baro_stat_t baro_init(void);

/**
 *	@name	baro_read_temp
 *	@brief	Read temperature from barometer
 *	@param	none
 *	@return Temperature in fixed point format t*100
 *			e.g. 12.34 deg would be 1234,
 *			INT32_MAX if error
 */
int32_t baro_read_temp(void);

/**
 *	@name	baro_read_press
 *	@brief	Read pressure from barometer
 *	@param	none
 *	@return Pressure in fixed point format P*1000
 *			e.g. 750.234 hPa would be 750234,
 *			UINT32_MAX if error
 */
uint32_t baro_read_press(void);

#endif /* INC_BARO_H_ */
