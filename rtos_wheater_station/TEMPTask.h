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
#include <stdbool.h>
#include <stdint.h>
/* Drivers Header files - fall back to driverlib for gpio*/
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <inc/hw_memmap.h>


#ifndef TEMP_TASK_H
#define TEMP_TASK_H

#define BOARD_THERMO_CLICK			 (0x5a)

/*! \fn TempFxn
 *  \brief Execute Temp Task
 *
 *
 *   \param arg0 void
 *   \param arg1 void
 *
*/
void TempFxn(UArg arg0, UArg arg1);

/*! \fn setup_Temp_Task
 *  \brief Setup Temp task
 *
 *  Setup Temp task
 *  Task has highest priority and receives 1kB of stack
 *
 *  \return always zero. In case of error the system halts.
*/
int setup_Temp_Task(void);

#endif
