/*
* CCSv6 project using TI-RTOS
*
*/

/*
* ======== StartBIOS.c ========
*/
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/hal/Timer.h>
#include <ti/sysbios/knl/Event.h>

/* TI-RTOS Header files */
#include <ti/drivers/UART.h>


/*Board Header files */
#include <Board.h>
#include <EK_TM4C1294XL.h>

/*application header files*/
#include <ctype.h>
#include <string.h>


#include <Blink_Task.h>
#include <UART_Task.h>

#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>/*supplies GPIO_PIN_x*/
#include <inc/hw_memmap.h>/*supplies GPIO_PORTx_BASE*/

#include "Lauflicht.h"
//#include "Altitude.h"
#include "BoosterPack.h"
extern int SetupAltiudeTask(BoosterPackType boosterPack);


int main(void)
{
    uint32_t ui32SysClock;
    //static led_descriptor_t led_desc[4];
    static uint32_t wait_ticks = 500;
    BoosterPackType boosterPackAltitude = BOOSTER_PACK_1;

    /* Call board init functions. */
    ui32SysClock = Board_initGeneral(120*1000*1000);
    (void)ui32SysClock; // We don't really need this (yet)

    Board_initGPIO();
    Board_initI2C();

    InitializeLed();

    /* led1 */
    /*Initialize+start Blink Task*/
    //(void) setup_Blink_Task(&wait_ticks);
    //System_printf("Created Blink Task\n");

    /*Initialize+start UART Task*/
    //(void) setup_UART_Task();
    //System_printf("Created UART Task\n");


    (void) SetupAltiudeTask(boosterPackAltitude);
    System_printf("Created Altitude Task\n");

    /* SysMin will only print to the console upon calling flush or exit */

    System_printf("Start BIOS\n");
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return 0;

}


