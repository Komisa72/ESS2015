/*
* Altitude.c
*
*  Created on: 03.01.2016
*      Author: amaierhofer
*/

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
//#include <xdc/runtime/Memory.h>


/* BIOS Header files */
//#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/I2C.h>


/* Board Header files */
#include <Board.h>
#include <EK_TM4C1294XL.h>
#include "BoosterPack.h"
#include "Altitude.h"

/* Globals */

/* Forward references */
void AltimeterInit(I2C_Handle i2c);



/**
 * Start Altimeter measurement.
 * param i2c handle to open I2C connection
 * return void
 */
void AltimeterInit(I2C_Handle i2c)
{
    I2C_Transaction i2cTransaction;
    bool flush = false;

    uint8_t txBuffer[2];
    uint8_t rxBuffer[2];

    // start one shot reading

    i2cTransaction.slaveAddress = BOARD_ALTIUDE_CLICK;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readBuf = NULL;//rxBuffer;
    i2cTransaction.readCount = 0;
    i2cTransaction.arg = 0;
    txBuffer[0] = MPL3115A2_CTRL_REG1;
    txBuffer[1] = 0b10111011; //  Altimeter selected, one shot mode, 128x oversampling (512 ms)

    if (!I2C_transfer(i2c, &i2cTransaction))
    {
        System_printf("I2C Bus fault\n");
        flush = true;
    }
    txBuffer[1] = 0b10111001; //  Clear oversampling bit
    if (!I2C_transfer(i2c, &i2cTransaction))
    {
        System_printf("I2C Bus fault\n");
        flush = true;
    }
    if (flush)
    {
    	System_flush();
    }
}

/**
 * Task function of altitude click.
 * param arg0 is BoosterPackType to identify which booster pack is used.
 * param arg1 always NULL.
 */
void AltitudeFunction(UArg arg0, UArg arg1)
{
    float result;
    I2C_Params i2cParams;
    unsigned int index;
    I2C_Handle i2c;

    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_100kHz;
    i2cParams.transferMode = I2C_MODE_BLOCKING;
    i2cParams.transferCallbackFxn = NULL;
    if ((BoosterPackType)arg0 == BOOSTER_PACK_1)
    {
        index = Board_I2C0;
    }
    else
    {
        // BOOSTER_PACK_2
        index = Board_I2C1;
    }
    i2c = I2C_open(index, &i2cParams);
    if (i2c == NULL) {
        System_abort("Error Initializing I2C\n");
    }
    else {
        System_printf("I2C Initialized!\n");
    }

    while (true)
    {
        AltimeterInit(i2c);
        Task_sleep(1000);
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
int SetupAltiudeTask(BoosterPackType boosterPack)
{
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
    taskAltitude = Task_create((Task_FuncPtr)AltitudeFunction, &taskAltitudeParams, &eb);
    if (taskAltitude == NULL) {
        System_abort("taskAltitude create failed");
    }

    return 0;
}

