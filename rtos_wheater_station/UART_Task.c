/*
 * UART_Task.c
 *
 *  Created on: 03.01.2016
 *      Author: amaierhofer
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

/* TI-RTOS Header files */
#include <driverlib/sysctl.h>
#include <driverlib/i2c.h>
#include <driverlib/ssi.h>
#include <driverlib/uart.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>/*supplies GPIO_PIN_x*/
#include <inc/hw_memmap.h>/*supplies GPIO_PORTx_BASE*/

#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

/*Board Header files */
#include <Board.h>
#include <EK_TM4C1294XL.h>

#include <ctype.h>
#include <string.h>
#include <math.h>

#include <Board.h>

#include "BoosterPack.h"
#include "ClockTask.h"

static int SetupTransferTask(void);


/**
 * /brief Reverses a string 'str' of length 'len'
 * /param str string to be reversed.
 * /param len of string str.
 */
static void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}

/**
 * /brief Converts an integer to string.
 *
 * Converts a given integer x to string str[].  d is the number
 * of digits required in output. If d is more than the number
 * of digits in x, then 0s are added at the beginning.
 *
 * /param x integer to be converted.
 * /param str where to store the result including terminating '\0'.
 * /param d minimum output width, filled up with 0.
 */
static int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

/**
 * /brief Converts a floating point number to string.
 * /param n number to be converted.
 * /param res where to store the resulting string including terminating 0.
 * /param afterpoint precision to be shown in result.
 */
static void ftoa(float n, char *res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 1);

    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot.
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}


/*
 *  ======== UART  ========
 *  Echo Characters recieved and show reception on Port N Led 0
 */
void UARTFxn(UArg arg0, UArg arg1) {

	char input;
	UART_Handle uart;
	UART_Params uartParams;
	const char echoPrompt[] = "\fEchoing characters:\r\n";

	/* Create a UART with data processing off. */
	UART_Params_init(&uartParams);
	uartParams.writeDataMode = UART_DATA_BINARY;
	uartParams.readDataMode = UART_DATA_BINARY;
	uartParams.readReturnMode = UART_RETURN_FULL;
	uartParams.readEcho = UART_ECHO_OFF;
	uartParams.baudRate = 9600;
	uart = UART_open(Board_UART0, &uartParams);

	if (uart == NULL) {
		System_abort("Error opening the UART");
	}

	UART_write(uart, echoPrompt, sizeof(echoPrompt));

	/* Loop forever echoing */
	while (1) {
		UART_read(uart, &input, 1);
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 1);
		UART_write(uart, &input, 1); //Remove this line to stop echoing!
		Task_sleep(5);
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
	}

}

/**
 *  /brief Setup UART tasks.
 *
 *  Setup UART tasks.
 *  Task has highest priority and receives 1kB of stack.
 *
  * /return Always 0. In case of error the system halts.
 */
int setup_UART_Task(void) {
	Task_Params taskUARTParams;
	Task_Handle taskUART;
	Error_Block eb;

	/* Enable and configure the peripherals used by the UART0 */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Enable and configure the peripherals used by the UART7 */
	SysCtlPeripheralEnable(GPIO_PORTC_BASE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
    GPIOPinConfigure(GPIO_PC4_U7RX);
    GPIOPinConfigure(GPIO_PC5_U7TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    UART_init();

	//Setup PortN LED1 activity signaling
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);

	Error_init(&eb);
	Task_Params_init(&taskUARTParams);
	taskUARTParams.stackSize = 1024;/*stack in bytes*/
	taskUARTParams.priority = 15;/*15 is default 16 is highest priority -> see RTOS configuration*/
	taskUART = Task_create((Task_FuncPtr) UARTFxn, &taskUARTParams, &eb);
	if (taskUART == NULL) {
		System_abort("TaskUART create failed");
	}
	SetupTransferTask();

	return (0);
}

/**
 * /brief Functions transfers read values from altitude/thermo click via uart7.
 *
 * /param arg0 not used.
 * /param arg1 not used.
 */
void TransferFunction(UArg arg0, UArg arg1) {
	TransferMessageType message;
	UInt firedEvents;
	UART_Handle uart7;
	UART_Params uart7Params;
    char result[20+1];
    int length;

    /* Create a UART with data processing off. */
    UART_Params_init(&uart7Params);
    uart7Params.writeDataMode = UART_DATA_BINARY;
    uart7Params.readDataMode = UART_DATA_BINARY;
    uart7Params.readReturnMode = UART_RETURN_FULL;
    uart7Params.readEcho = UART_ECHO_OFF;
    uart7Params.baudRate = 9600;
    uart7 = UART_open(Board_UART3, &uart7Params);

    if (uart7 == NULL) {
        System_abort("Error opening the UART");
    }

	while (true) {
		firedEvents = Event_pend(transferEvent, Event_Id_NONE, /* andMask = 0 */
		TRANSFER_MESSAGE_EVENT, /* orMask */
		BIOS_WAIT_FOREVER); /* timeout */
		if (firedEvents & TRANSFER_MESSAGE_EVENT) {
			/* Get the posted message.
			 * Mailbox_pend() will not block since Event_pend()
			 * has guaranteed that a message is available.
			 * Notice that the special BIOS_NO_WAIT
			 * parameter tells Mailbox that Event_pend()
			 * was used to acquire the available message.
			 */
			Mailbox_pend(transferMailbox, &message, BIOS_NO_WAIT);
			switch (message.kind) {
			case TRANSFER_TEMPERATURE:
				result[0] = TRANSFER_TEMPERATURE;
			    ftoa(message.value, &result[1], TEMPERATURE_PRECISION);
			    length = strlen(result) + 1;
				UART_write(uart7, &result[0], length);
				break;
			case TRANSFER_PRESSURE:
				result[0] = TRANSFER_PRESSURE;
			    ftoa(message.value, &result[1], PRESSURE_PRECISION);
			    length = strlen(result) + 1;
				UART_write(uart7, &result[0], length);
				break;
			case TRANSFER_ALTITUDE:
				result[0] = TRANSFER_ALTITUDE;
			    ftoa(message.value, &result[1], ALTITUDE_PRECISION);
			    length = strlen(result) + 1;
				UART_write(uart7, &result[0], length);
				break;
			default:
				// unknown, nothing special
				continue;
				break;
			}
			System_printf("value %s \n", result);
			System_flush();
		}
	}
}

/*
 * /brief Setup task for transferring data via uart.
 * /return Always 0. In case of error the system halts.
 */
static int SetupTransferTask(void) {
	Task_Params taskTransferParams;
	Task_Handle taskTransfer;
	Error_Block eb;

	Error_init(&eb);
	Task_Params_init(&taskTransferParams);
	taskTransferParams.stackSize = 1024;/*stack in bytes*/
	taskTransferParams.priority = 15;/*15 is default 16 is highest priority -> see RTOS configuration*/
	taskTransfer = Task_create((Task_FuncPtr) TransferFunction,
			&taskTransferParams, &eb);
	if (taskTransfer == NULL) {
		System_abort("TaskTransfer create failed");
	}
	return 0;

}

