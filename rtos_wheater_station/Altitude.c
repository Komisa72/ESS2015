/*
* Altitude.c
*
*  Created on: 03.01.2016
*      Author: amaierhofer
*/

/*
*  setup task function
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


/*Board Header files */
#include <Board.h>
#include <EK_TM4C1294XL.h>
#include "BoosterPack.h"
#include "Altitude.h"

static I2C_Handle i2c;

void Altimeter_Init()
{
    I2C_Transaction i2cTransaction;

    uint8_t txBuffer[2];
    uint8_t rxBuffer[2];

    // read device id from register 0xC
    i2cTransaction.slaveAddress = MPL3115A2_WRITE_ADDRESS;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;//rxBuffer;
    i2cTransaction.readCount = 1;
    i2cTransaction.arg = 0;
    txBuffer[0] = 0x0C;
    rxBuffer[0] = 0x0;

    if (I2C_transfer(i2c, &i2cTransaction))
    {
        System_printf("Init ok\n");
    }
    else
    {
        System_printf("I2C Bus fault\n");
    }

    // start one shot reading

    i2cTransaction.slaveAddress = MPL3115A2_WRITE_ADDRESS;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readBuf = NULL;//rxBuffer;
    i2cTransaction.readCount = 0;
    i2cTransaction.arg = 0;
    txBuffer[0] = MPL3115A2_CTRL_REG1;
    txBuffer[1] = 0b10111011; //  Altimeter selected, one shot mode, 128x oversampling (512 ms)

    if (I2C_transfer(i2c, &i2cTransaction))
    {
        System_printf("Init ok\n");
    }
    else
    {
        System_printf("I2C Bus fault\n");
    }

    //MPL3115A2_Write(_CTRL_REG1, 0b10111011);
    //    MPL3115A2_Write(_CTRL_REG1, 0b10111001);     // Clear oversampling bit
}

void AltitudeFunction(UArg arg0, UArg arg1)
{

    uint16_t       	ambtemp;
    uint16_t       	objtemp;
    float			result;
    uint8_t         txBuffer[1];
    uint8_t         rxBuffer[2];

    I2C_Params      i2cParams;
    I2C_Transaction i2cTransaction;
    unsigned int index;

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

    Altimeter_Init();


    for(;;){
#if 0
        /* Point to the T ambient register and read its 2 bytes */
        txBuffer[0] = 0x06;
        i2cTransaction.slaveAddress = Board_Thermo_CLICK;
        i2cTransaction.writeBuf = txBuffer;
        i2cTransaction.writeCount = 1;
        i2cTransaction.readBuf = rxBuffer;
        i2cTransaction.readCount = 2;

        if (I2C_transfer(i2c, &i2cTransaction)) {
            /* Extract degrees C from the received data;*/
            ambtemp = ((rxBuffer[1] << 8 ) | rxBuffer[0]);

            result = ((ambtemp / 50)- 273.15);
            System_printf("AmbTemp: %f (C)\n", result);

        }
        else {
            System_printf("I2C Bus fault\n");
        }

        /* Take Object Temp and print it out onto the console */

        txBuffer[0] = 0x07; /*Set to Object temperature register */

        if (I2C_transfer(i2c, &i2cTransaction)) {
            /* Extract degrees C from the received data;*/
            objtemp = ((rxBuffer[1] << 8 ) | rxBuffer[0]);

            result = ((objtemp / 50)- 273.15);
            System_printf("Objektemperature: %f (C)\n", result);

        }
        else {
            System_printf("I2C Bus fault\n");
        }
        System_flush();
#endif
        Task_sleep(1000);
    }


#if 0

    //led_descriptor_t *led_desc = (led_descriptor_t *)arg0;
    uint32_t waitTicks = 500;
    /* arg1 is NULL in my case */
    /*gpio driverlib api uses same bit pattern for gpio mask and value*/
    //uint8_t ui8val = (uint8_t)led_desc->led;

    while(1) {
        //LedHandler();

        //ui8val ^= (uint8_t)led_desc->led;//initially off
        //GPIOPinWrite (led_desc->port_base, led_desc->led, ui8val);
        Task_sleep(waitTicks);
    }
#endif

#ifdef DEBUG
    /* Deinitialized I2C */
    I2C_close(i2c);
    System_printf("I2C closed!\n");

    System_flush();
#endif



}



int SetupAltiudeTask(BoosterPackType boosterPack)
{
    Task_Params taskAltitudeParams;
    Task_Handle taskAltitude;
    Error_Block eb;

    /* Create altitude task with priority 15*/
    Error_init(&eb);
    Task_Params_init(&taskAltitudeParams);
    taskAltitudeParams.stackSize = 1024;/*stack in bytes*/
    taskAltitudeParams.priority = 15;/*15 is default 16 is highest priority -> see RTOS configuration*/
    taskAltitudeParams.arg0 = (UArg) boosterPack;
    taskAltitudeParams.arg1 = NULL;
    taskAltitude = Task_create((Task_FuncPtr)AltitudeFunction, &taskAltitudeParams, &eb);
    if (taskAltitude == NULL) {
        System_abort("taskAltitud create failed");
    }

    return 0;
}

