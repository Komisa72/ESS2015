/*
 * ClockTask.c
 *
 *  Created on: 05.01.2016
 *      Author: amaierhofer
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

#include "ClockTask.h"
#include "BoosterPack.h"

Event_Handle measureEvent;

void ClockFunction(UArg arg0)
{
	UInt eventId = 0;
#if USE_THERMO_CLICK
    eventId |= MEASURE_THERMO_EVENT;
#endif
#if USE_ALTITUDE_CLICK
    eventId |= MEASURE_ALTITUDE_EVENT;
#endif

	Event_post(measureEvent, eventId);

}


int SetupClockTask(uint32_t wait_ticks)
{
	Clock_Params clockParams;
	Clock_Handle myClock;
	Error_Block eb;

	Error_init(&eb);
	measureEvent = Event_create(NULL, &eb);
	if (measureEvent == NULL)
	{
		System_abort("Event create failed");
	}

	Clock_Params_init(&clockParams);
	if (wait_ticks < 1000)
	{
		wait_ticks = 1000;  // at minimum 1s
	}

	clockParams.period = wait_ticks;
	clockParams.startFlag = TRUE;
	clockParams.arg = NULL;

	myClock = Clock_create((Clock_FuncPtr)ClockFunction, wait_ticks, &clockParams, &eb);
	if (myClock == NULL)
	{
		System_abort("Clock create failed");
	}
}
