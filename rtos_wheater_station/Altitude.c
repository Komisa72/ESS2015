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
//#include <xdc/cfg/global.h>
//#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
//#include <xdc/runtime/Memory.h>


/* BIOS Header files */
//#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>


/*Board Header files */
#include <Board.h>
#include <EK_TM4C1294XL.h>

void AltitudeFunction(UArg arg0, UArg arg1)
{
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

}



int SetupAltiudeTask(uint32_t* wait_ticks)
{
	Task_Params taskAltitudeParams;
	Task_Handle taskAltitude;
	Error_Block eb;

    /* Create altitude task with priority 15*/
    Error_init(&eb);
    Task_Params_init(&taskAltitudeParams);
    taskAltitudeParams.stackSize = 1024;/*stack in bytes*/
    taskAltitudeParams.priority = 15;/*15 is default 16 is highest priority -> see RTOS configuration*/
    taskAltitudeParams.arg0 = (UArg) wait_ticks;
    taskAltitudeParams.arg1 = NULL;
    taskAltitude = Task_create((Task_FuncPtr)AltitudeFunction, &taskAltitudeParams, &eb);
    if (taskAltitude == NULL) {
    	System_abort("taskAltitud create failed");
    }

    return 0;
}

