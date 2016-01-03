/*
 *  ======== UART_Task.c ========
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
#include <Blink_Task.h>
#include <Board.h>
#include <EK_TM4C1294XL.h>


#include <ctype.h>
#include <string.h>

/*
 *  ======== UART  ========
 *  Echo Characters recieved and show reception on Port N Led 0
 */
void UARTFxn(UArg arg0, UArg arg1)
{

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


/*
 *  setup task function
 */
int setup_UART_Task(void)
{
	Task_Params taskUARTParams;
	Task_Handle taskUART;
	Error_Block eb;


	/* Enable and configure the peripherals used by the UART0 */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UART_init();

	//Setup PortN LED1 activity signaling
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE,GPIO_PIN_0);
	
    /* Create Blink task with priority 15*/
    Error_init(&eb);
    Task_Params_init(&taskUARTParams);
    taskUARTParams.stackSize = 1024;/*stack in bytes*/
    taskUARTParams.priority = 15;/*15 is default 16 is highest priority -> see RTOS configuration*/
    taskUART = Task_create((Task_FuncPtr)UARTFxn, &taskUARTParams, &eb);
    if (taskUART == NULL) {
    	System_abort("TaskUART create failed");
    }

    return (0);
}
