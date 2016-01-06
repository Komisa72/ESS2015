/*
 * ClockTask.h
 *
 *  Created on: 05.01.2016
 *      Author: amaierhofer
 */

#ifndef CLOCKTASK_H_
#define CLOCKTASK_H_

#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Mailbox.h>

/* typedef */

typedef enum TransferMessageKindEnum {
	TRANSFER_TEMPERATURE,
	TRANSFER_PRESSURE,
	TRANSFER_ALTITUDE,
	TRASFER_HEIGHT_ALARM,
} TransferMessageKindType;

typedef struct TransferMessageStruct {
	TransferMessageKindType kind;
	float value;
} TransferMessageType;

/* defines */
#define MEASURE_THERMO_EVENT Event_Id_00
#define MEASURE_ALTITUDE_EVENT Event_Id_00

#define TRANSFER_MESSAGE_EVENT Event_Id_00
#define TRANSFER_MAILBOX_SIZE 10

// 4 digits precision for temperature
#define TEMPERATURE_PRECISION 4

#define ALTITUDE_PRECISION 3
#define PRESSURE_PRECISION 1

/* global */
extern Event_Handle measureAltitudeEvent; // trigger measurement of thermo/altitude click
extern Event_Handle measureThermoEvent; // trigger measurement of thermo/altitude click
extern Event_Handle transferEvent; // trigger transfer of read data
extern Mailbox_Handle transferMailbox; // contains data to be transferred

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
