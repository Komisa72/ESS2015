/*
 * Altitude.h
 *
 *  Created on: 03.01.2016
 *      Author: amaierhofer
 */

#ifndef ALTITUDE_H_
#define ALTITUDE_H_

// MPL3115A2 register definitions

#define _OUT_T_MSB   0x04    // Temperature data (MSB)
#define _OUT_T_LSB   0x05    // Temperature data (LSB)
#define _PT_DATA_CFG 0x13    // Data event flag configuration
#define _BAR_IN_MSB  0x14    // Barometric input for Altitude calculation
#define _BAR_IN_LSB  0x15    // Barometric input for Altitude calculation
#define _OFFH        0x2D    // Altitude Data User Offset Register

/* IIC bus address of Altitude Click */
#define BOARD_ALTIUDE_CLICK 0x60

/* register addresses */
#define MPL3115A2_OUT_P_MSB   0x01    // Pressure/Altitude data (MSB)
#define MPL3115A2_OUT_P_CSB   0x02    // Pressure/Altitude data (middle)
#define MPL3115A2_OUT_P_LSB   0x03    // Pressure/Altitude data (LSB)
#define MPL3115A2_CTRL_REG1   0x26    // Control register 1

/* Vienna height */
#define START_ALTITUDE 156


#define STANDBY_BIT 0
#define ALT  7 // Altimeter/barometer switch bit

#define STANDBY_BIT_MASK  0xFE // 0b11111110 mask

typedef struct ReadDataStruct
{
	uint8_t low_byte;
	uint8_t middle_byte;
	uint8_t high_byte;
} ReadDataType;


#endif /* ALTITUDE_H_ */
