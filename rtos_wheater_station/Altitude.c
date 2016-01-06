/*
 * Altitude.c
 *
 *  Created on: 03.01.2016
 *      Author: amaierhofer
 */
#include <math.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
//#include <xdc/runtime/Memory.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/I2C.h>

/* Board Header files */
#include <Board.h>
#include <EK_TM4C1294XL.h>
#include "BoosterPack.h"
#include "Altitude.h"
#include "ClockTask.h"

/* Defines */
#define ClearBit(reg, bit)       reg &= (~(1<<(bit)))
#define SetBit(reg, bit)         reg |= (1<<(bit))

/* Globals */
// Fractional bit values for altitude
static const float ALT_FRAC_B7 = 0.5;    // Fractional value for bit 7
static const float ALT_FRAC_B6 = 0.25;   // Fractional value for bit 6
static const float ALT_FRAC_B5 = 0.125;  // Fractional value for bit 5
static const float ALT_FRAC_B4 = 0.0625; // Fractional value for bit 4

// Fractional bit values for pressure
static const float PRES_FRAC_B5 = 0.5;   // Fractional value for bit 5
static const float PRES_FRAC_B4 = 0.25;  // Fractional value for bit 4

/* Forward references */
void ReadData(I2C_Handle i2c, ReadDataType* pread);
float PressureRead(I2C_Handle i2c);
void SwitchToStandby(I2C_Handle i2c);

/**
 * /brief Write 1 byte to the given address.
 * /param i2c handle to open I2C connection.
 * /param register address where to write.
 * /param data_ data to be written
 */
void MPL3115A2_Write(I2C_Handle i2c, uint8_t address, uint8_t value) {
	I2C_Transaction i2cTransaction;
	bool flush = false;
	uint8_t txBuffer[2];

	i2cTransaction.slaveAddress = BOARD_ALTIUDE_CLICK;
	i2cTransaction.writeBuf = txBuffer;
	i2cTransaction.writeCount = 2;
	i2cTransaction.readBuf = NULL;
	i2cTransaction.readCount = 0;
	i2cTransaction.arg = 0;
	txBuffer[0] = address;
	txBuffer[1] = value;

	if (!I2C_transfer(i2c, &i2cTransaction)) {
		System_printf("I2C Bus fault\n");
	}
	if (flush) {
		System_flush();
	}

}

// Read from MPL3115A2 sensor
uint8_t MPL3115A2_Read(I2C_Handle i2c, uint8_t address) {
	I2C_Transaction i2cTransaction;
	bool flush = false;

	uint8_t txBuffer[2];
	uint8_t rxBuffer[2];

	i2cTransaction.slaveAddress = BOARD_ALTIUDE_CLICK;
	i2cTransaction.writeBuf = txBuffer;
	i2cTransaction.writeCount = 1;
	i2cTransaction.readBuf = rxBuffer;
	i2cTransaction.readCount = 1;
	i2cTransaction.arg = 0;
	txBuffer[0] = address;

	if (!I2C_transfer(i2c, &i2cTransaction)) {
		System_printf("I2C Bus fault\n");
	}
	if (flush) {
		System_flush();
	}
	return rxBuffer[0];

}

/**
 * Start Altimeter measurement.
 * param i2c handle to open I2C connection
 * return void
 */
void AltimeterInit(I2C_Handle i2c) {
	I2C_Transaction i2cTransaction;
	bool flush = false;
	uint8_t txBuffer[2];

	// start one shot reading
	i2cTransaction.slaveAddress = BOARD_ALTIUDE_CLICK;
	i2cTransaction.writeBuf = txBuffer;
	i2cTransaction.writeCount = 2;
	i2cTransaction.readBuf = NULL;  //rxBuffer;
	i2cTransaction.readCount = 0;
	i2cTransaction.arg = 0;
	txBuffer[0] = MPL3115A2_CTRL_REG1;
	txBuffer[1] = 0b10111011; //  Altimeter selected, one shot mode, 128x oversampling (512 ms)

	if (!I2C_transfer(i2c, &i2cTransaction)) {
		System_printf("I2C Bus fault\n");
		flush = true;
	}
	txBuffer[1] = 0b10111001; //  Clear oversampling bit
	if (!I2C_transfer(i2c, &i2cTransaction)) {
		System_printf("I2C Bus fault\n");
		flush = true;
	}
	if (flush) {
		System_flush();
	}
}

/**
 * // switches device to Standby mode for making changes to control registers
 * param i2c handle to open I2C connection.
 */
void SwitchToStandby(I2C_Handle i2c) {
	uint8_t CTRL_REG_1_DATA;
	CTRL_REG_1_DATA = MPL3115A2_Read(i2c, MPL3115A2_CTRL_REG1); //read control register
	ClearBit(CTRL_REG_1_DATA, STANDBY_BIT); // reset Standby bit (switch to Standby mode)
	MPL3115A2_Write(i2c, MPL3115A2_CTRL_REG1, CTRL_REG_1_DATA); // write to control register
}

void SwitchToActive(I2C_Handle i2c) // switches device to active mode
{
	uint8_t CTRL_REG_1_DATA;
	CTRL_REG_1_DATA = MPL3115A2_Read(i2c, MPL3115A2_CTRL_REG1); //read control register
	SetBit(CTRL_REG_1_DATA, STANDBY_BIT); // set Standby bit (switch to active mode)
	MPL3115A2_Write(i2c, MPL3115A2_CTRL_REG1, CTRL_REG_1_DATA); // write to control register
}

void SwitchToAltimeter(I2C_Handle i2c) // switch device to altimeter mode
{
	uint8_t CTRL_REG_1_DATA;
	SwitchToStandby(i2c);
	MPL3115A2_Write(i2c, _PT_DATA_CFG, 0x07); // Enable Data Flags in PT_DATA_CFG
	CTRL_REG_1_DATA = MPL3115A2_Read(i2c, MPL3115A2_CTRL_REG1); //read control register
	SetBit(CTRL_REG_1_DATA, ALT);                   // Set Altimeter bit
	SetBit(CTRL_REG_1_DATA, STANDBY_BIT); // set Standby bit (switch to active mode)
	MPL3115A2_Write(i2c, MPL3115A2_CTRL_REG1, CTRL_REG_1_DATA); // write to control register
}

void SwitchToBarometer(I2C_Handle i2c) // switch device to barometer mode
{
	unsigned short CTRL_REG_1_DATA;
	SwitchToStandby(i2c);
	MPL3115A2_Write(i2c, _PT_DATA_CFG, 0x07); // Enable Data Flags in PT_DATA_CFG
	CTRL_REG_1_DATA = MPL3115A2_Read(i2c, MPL3115A2_CTRL_REG1); //read control register
	ClearBit(CTRL_REG_1_DATA, ALT);                 // Reset Altimeter bit
	SetBit(CTRL_REG_1_DATA, STANDBY_BIT); // set Standby bit (switch to active mode)
	MPL3115A2_Write(i2c, MPL3115A2_CTRL_REG1, CTRL_REG_1_DATA); // write to control register
}

/**
 * Calibrate the altimeter.
 * param i2c handle to open I2C connection.
 */
void MPL3115A2Calibrate(I2C_Handle i2c) {
	float current_pressure;
	float calibration_pressure;
	float sea_pressure;
	int i;
	ReadDataType read;

	// Altitude offset set to 0
	MPL3115A2_Write(i2c, _OFFH, 0);

	// Clear value
	calibration_pressure = 0;

	// Calculate current pressure level
	for (i = 0; i < 8; i++) {
		MPL3115A2_Write(i2c, MPL3115A2_CTRL_REG1, 0b00111011); // One shot mode, 128x oversampling (512 ms)
		MPL3115A2_Write(i2c, MPL3115A2_CTRL_REG1, 0b00111001); // Clear oversampling bit
		Task_sleep(550);                     // Wait for sensor to read pressure
		//ReadData(i2c, &read); // Read sensor data
		// Read barometric pressure
		calibration_pressure = calibration_pressure + PressureRead(i2c);
	}
	// Find average value of current pressure level readings
	current_pressure = calibration_pressure / 8;

	// Calculate barometric pressure at mean sea level based on a starting altitude
	sea_pressure = current_pressure
			/ pow(1 - START_ALTITUDE * 0.0000225577, 5.255877);

	// Calibrate the sensor according to the sea level pressure for the current measurement location (2 Pa per LSB) :
	MPL3115A2_Write(i2c, _BAR_IN_MSB, (uint8_t) (sea_pressure / 2) >> 8);
	MPL3115A2_Write(i2c, _BAR_IN_LSB, (uint8_t) (sea_pressure / 2) & 0xFF);
}

/**
   /brief Set Output Sample Rate
   The OSR_Value is set to 0 - 7 corresponding with Ratios 1 - 128
*/
void SetOSR(I2C_Handle i2c, unsigned char value)
{
   uint8_t control_register;
   SwitchToStandby(i2c);
   control_register = MPL3115A2_Read(i2c, MPL3115A2_CTRL_REG1);   //read control register
   if(value < 8)
   {
      value <<= 3;
      MPL3115A2_Write(i2c, MPL3115A2_CTRL_REG1, control_register | value);
   }
}

/**
 * Common initialisation.
 */
void MPL3115A2Init(I2C_Handle i2c) {
	SetOSR(i2c, 7);  // highest oversampling 512 ms
	MPL3115A2_Write(i2c, _PT_DATA_CFG, 0x07); // Enable Data Flags in PT_DATA_CFG
}

/**
 * Read the pressure/altimeter value.
 */
void ReadData(I2C_Handle i2c, ReadDataType* pread) {
    uint8_t lowTemp;
	uint8_t highTemp;

	pread->low_byte = MPL3115A2_Read(i2c, MPL3115A2_OUT_P_LSB);
	pread->middle_byte = MPL3115A2_Read(i2c, MPL3115A2_OUT_P_CSB);
	pread->high_byte = MPL3115A2_Read(i2c, MPL3115A2_OUT_P_MSB);

	highTemp = MPL3115A2_Read(i2c, _OUT_T_MSB);
	lowTemp = MPL3115A2_Read(i2c, _OUT_T_LSB);


}

// Read Barometric Pressure Value
float PressureRead(I2C_Handle i2c) {
	ReadDataType read;
	unsigned long pressure_int;
	float pressure_frac_value = 0;

	// Read data registers
	ReadData(i2c, &read);

	// Calculate the integer part of the barometric pressure value
	pressure_int = read.low_byte | ((read.middle_byte << 8))
			| ((unsigned long) read.high_byte << 16);
	pressure_int = pressure_int >> 6;

	// Clear the fractional part of the barometric pressure value

	// Calculate the fractional part of the barometric pressure value
	if (read.low_byte & 0x20 == 1) {
		pressure_frac_value = pressure_frac_value + PRES_FRAC_B5;
	}
	if (read.low_byte & 0x10 == 1) {
		pressure_frac_value = pressure_frac_value + PRES_FRAC_B4;
	}
	// Sum the integer part and fractional part of the pressure value
	return (float) pressure_int + pressure_frac_value;
}

/**
 * Read out the altitude value from registers in meter.
 * param i2c handle to open I2C connection.
 * return Altitude in meter.
 */
float AltitudeRead(I2C_Handle i2c) {
	ReadDataType read;
	float altitude_value;
	float altitude_frac_value = 0;
	// Read data registers
	read.high_byte = 0;
	read.middle_byte = 0;
	ReadData(i2c, &read);

	// Calculate the integer part of the altitude value
	altitude_value = (short) ((read.middle_byte
			| ((unsigned short) read.high_byte << 8)));

	// Calculate the fractional part of the altitude value
	if (read.low_byte & 0x80 == 1) {
		altitude_frac_value = altitude_frac_value + ALT_FRAC_B7;
	}
	if (read.low_byte & 0x40 == 1) {
		altitude_frac_value = altitude_frac_value + ALT_FRAC_B6;
	}
	if (read.low_byte & 0x20 == 1) {
		altitude_frac_value = altitude_frac_value + ALT_FRAC_B5;
	}
	if (read.low_byte & 0x10 == 1) {
		altitude_frac_value = altitude_frac_value + ALT_FRAC_B4;
	}
	// Sum the integer part and fractional part of the altitude value
	return altitude_value = altitude_value + altitude_frac_value;
}

/**
 * Task function of altitude click.
 * param arg0 is BoosterPackType to identify which booster pack is used.
 * param arg1 always NULL.
 */
void AltitudeFunction(UArg arg0, UArg arg1) {
	I2C_Params i2cParams;
	unsigned int index;
	I2C_Handle i2c;
	float altitude;
	UInt eventFired;
	TransferMessageType altimeter;
	TransferMessageType barometer;

	altimeter.kind = TRANSFER_ALTITUDE;
	barometer.kind = TRANSFER_PRESSURE;
	barometer.value = 3;

	/* Create I2C for usage */
	I2C_Params_init(&i2cParams);
	i2cParams.bitRate = I2C_100kHz;
	i2cParams.transferMode = I2C_MODE_BLOCKING;
	i2cParams.transferCallbackFxn = NULL;
	if ((BoosterPackType) arg0 == BOOSTER_PACK_1) {
		index = Board_I2C0;
	} else {
		// BOOSTER_PACK_2
		index = Board_I2C1;
	}
	i2c = I2C_open(index, &i2cParams);
	if (i2c == NULL) {
		System_abort("Error Initializing I2C\n");
	} else {
		System_printf("I2C Initialized!\n");
	}
#if USE_ALTITUDE_CLICK
	MPL3115A2Init(i2c);
	//MPL3115A2Calibrate(i2c);
#endif

	while (true) {
		// trigger measurement only if event is set
		eventFired = Event_pend(measureEvent, Event_Id_NONE, MEASURE_ALTITUDE_EVENT, BIOS_WAIT_FOREVER);
		SwitchToAltimeter(i2c);
		AltimeterInit(i2c);
		Task_sleep(550);
		altitude = AltitudeRead(i2c);
		altimeter.value = altitude;

		/* implicitly posts TRANSFER_MESSAGE_EVENT to transferEvent */
		Mailbox_post(transferMailbox, &altimeter, BIOS_WAIT_FOREVER);

		Mailbox_post(transferMailbox, &barometer, BIOS_WAIT_FOREVER);

	}

#ifdef DEBUG
	/* Deinitialized I2C */
	I2C_close(i2c);
	System_printf("I2C closed!\n");

	System_flush();
#endif

}

/**
 * Initialise altitude task and start it.
 * param boosterPack which booster pack is used for altitude click.
 * return always 0.
 */
int SetupAltiudeTask(BoosterPackType boosterPack) {
	Task_Params taskAltitudeParams;
	Task_Handle taskAltitude;
	Error_Block eb;

	/* Create altitude task with priority 15 */
	Error_init(&eb);
	Task_Params_init(&taskAltitudeParams);
	taskAltitudeParams.stackSize = 1024; /*stack in bytes*/
	taskAltitudeParams.priority = 15;/*15 is default 16 is highest priority -> see RTOS configuration*/
	taskAltitudeParams.arg0 = (UArg) boosterPack;
	taskAltitudeParams.arg1 = NULL;
	taskAltitude = Task_create((Task_FuncPtr) AltitudeFunction,
			&taskAltitudeParams, &eb);
	if (taskAltitude == NULL) {
		System_abort("taskAltitude create failed");
	}

	return 0;
}

