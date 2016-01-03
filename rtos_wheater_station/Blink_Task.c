/*
 *  ======== Blink_Task.c ========
 *  Author: Michael Kramer / Matthias Wenzl
 */
#include <stdbool.h>
#include <inc/hw_memmap.h>


/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>


/*Board Header files */
#include <Blink_Task.h>
#include <Board.h>
#include <EK_TM4C1294XL.h>


#include <ctype.h>
#include <string.h>

#include "Lauflicht.h"
/*
 *  ======== Blink  ========
 *  Perform Blink Operation on led given at arg0
 */
void BlinkFxn(UArg arg0, UArg arg1)
{
	//led_descriptor_t *led_desc = (led_descriptor_t *)arg0;
	uint32_t wait_ticks = *(uint32_t*) arg0;
	/* arg1 is NULL in my case */
	/*gpio driverlib api uses same bit pattern for gpio mask and value*/
	//uint8_t ui8val = (uint8_t)led_desc->led;

	while(1) {
       LedHandler();

	//ui8val ^= (uint8_t)led_desc->led;//initially off
	//GPIOPinWrite (led_desc->port_base, led_desc->led, ui8val);
	Task_sleep(wait_ticks);

	}

}



/*
 *  setup task function
 */
int setup_Blink_Task(uint32_t* wait_ticks)
{
#if 0
	Task_Params taskLedParams;
	Task_Handle taskLed;
	Error_Block eb;

    /*configure gpio port_base according to led*/
	//GPIOPinTypeGPIOOutput(led_desc->port_base, led_desc->led);

    /* Create Blink task with priority 15*/
    Error_init(&eb);
    Task_Params_init(&taskLedParams);
    taskLedParams.stackSize = 1024;/*stack in bytes*/
    taskLedParams.priority = 15;/*15 is default 16 is highest priority -> see RTOS configuration*/
    taskLedParams.arg0 = (UArg) wait_ticks;
    taskLedParams.arg1 = NULL; // (UArg) wait_ticks;
    taskLed = Task_create((Task_FuncPtr)BlinkFxn, &taskLedParams, &eb);
    if (taskLed == NULL) {
    	System_abort("TaskLed create failed");
    }
#endif
    return (0);
}