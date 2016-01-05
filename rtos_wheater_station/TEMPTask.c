/*
 *  ======== TEMP_Task.c ========
 *  Author: Thomas Schmid
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>

/* Example/Board Header files */
#include "Board.h"
#include "TempTask.h"

/*
 *  ======== TEMP  ========
 *  Get Ambient Temperature once and Object Temperature continously
 */

void TempFxn(UArg arg0, UArg arg1) {
	uint16_t ambtemp;
	uint16_t objtemp;
	float result;
	uint8_t txBuffer[1];
	uint8_t rxBuffer[2];
	I2C_Handle i2c;
	I2C_Params i2cParams;
	I2C_Transaction i2cTransaction;

	/* Create I2C for usage */
	I2C_Params_init(&i2cParams);
	i2cParams.bitRate = I2C_100kHz;
	i2c = I2C_open(Board_I2C_TMP, &i2cParams);
	if (i2c == NULL) {
		System_abort("Error Initializing I2C\n");
	} else {
		System_printf("I2C Initialized!\n");
	}
	for (;;) {
		/* Point to the T ambient register and read its 2 bytes */
		txBuffer[0] = 0x06;
		i2cTransaction.slaveAddress = BOARD_THERMO_CLICK;
		i2cTransaction.writeBuf = txBuffer;
		i2cTransaction.writeCount = 1;
		i2cTransaction.readBuf = rxBuffer;
		i2cTransaction.readCount = 2;

		if (I2C_transfer(i2c, &i2cTransaction)) {
			/* Extract degrees C from the received data;*/
			ambtemp = ((rxBuffer[1] << 8) | rxBuffer[0]);

			result = ((ambtemp / 50) - 273.15);
			System_printf("AmbTemp: %f (C)\n", result);

		} else {
			System_printf("I2C Bus fault\n");
		}

		/* Take Object Temp and print it out onto the console */

		txBuffer[0] = 0x07; /*Set to Object temperature register */

		if (I2C_transfer(i2c, &i2cTransaction)) {
			/* Extract degrees C from the received data;*/
			objtemp = ((rxBuffer[1] << 8) | rxBuffer[0]);

			result = ((objtemp / 50) - 273.15);
			System_printf("Objektemperature: %f (C)\n", result);

		} else {
			System_printf("I2C Bus fault\n");
		}
		System_flush();

		Task_sleep(1000);
	}

#ifdef DEBUG
	/* Deinitialized I2C */
	I2C_close(i2c);
	System_printf("I2C closed!\n");

	System_flush();
#endif
}

/*
 *  setup task function
 */
int setup_Temp_Task(void) {
	/* Setup Task and create it */
	Task_Params task0Params;
	Task_Handle task0Handle;
	Task_Params_init(&task0Params);
	task0Params.stackSize = 2048;
	task0Params.arg0 = NULL;
	task0Params.arg1 = NULL;
	task0Params.priority = 14;
	task0Handle = Task_create((Task_FuncPtr) TempFxn, &task0Params, NULL);
	if (task0Handle == NULL) {
		System_abort("Error creating task");
	}
	return 0;
}

