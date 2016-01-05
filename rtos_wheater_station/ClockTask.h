/*
 * ClockTask.h
 *
 *  Created on: 05.01.2016
 *      Author: amaierhofer
 */

#ifndef CLOCKTASK_H_
#define CLOCKTASK_H_

#include <ti/sysbios/knl/Event.h>

extern Event_Handle measureEvent;

#define MEASURE_THERMO_EVENT Event_Id_00
#define MEASURE_ALTITUDE_EVENT Event_Id_01

/** \fn setupClockTask
 *  \brief Setup clock task
 *
 *  Setup clock task
 *  Task has highest priority and receives 1kB of stack
 *
 *  \param time to wait for new measurement of temperature, pressure etc.
 *
 *  \return always zero. In case of error the system halts.
*/
int SetupClockTask(uint32_t wait_ticks);


#endif /* CLOCKTASK_H_ */
