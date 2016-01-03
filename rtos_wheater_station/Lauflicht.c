/*
* Lauflicht.c
*
*  Created on: 11.11.2015
*      Author: amaierhofer
*/

#include <stdbool.h> /*driverlib header requires stdbool.h to be included a first header file before any driverlib header*/
#include <stdint.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>/*supplies GPIO_PIN_x*/
#include <inc/hw_memmap.h>/*supplies GPIO_PORTx_BASE*/
#include <driverlib/sysctl.h>

#include "Lauflicht.h"

/* Bit masks for switching leds */
#define LEDALL_OFF  (0x0)
#define LED1_ON   (0x1)
#define LED2_ON   (0x2)
#define LED3_ON   (0x4)
#define LED4_ON   (0x8)


static uint8_t s_led_loop;
static uint8_t s_state[4];
static int s_led_count;

void InitializeOutput(uint32_t port, uint8_t pin);
void Led(uint8_t mask);

void InitializeLed()
{
    /* init  static variables */
    s_led_count = sizeof(s_state) / sizeof(s_state[0]);
    s_state[0] = LED1_ON;
    s_state[1] = LED2_ON;
    s_state[2] = LED3_ON;
    s_state[3] = LED4_ON;

    //uint32_t strength;
    //uint32_t pinType;

    /*activate gpio port n, f, j */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);

    /*configure led 1 and 2 GPIO_PIN_0 / 1 on port N as std pin*/
    InitializeOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /*configure led 3 and 4 GPIO_PIN_0 / 4 on port F as std pin*/
    InitializeOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);

    /* configure switch 1 with pull up as input on GPIO_PIN_0 as pull-up pin */
    //GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0);
    //GPIOPadConfigGet(GPIO_PORTJ_BASE, GPIO_PIN_0, &strength, &pinType);
    //GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0, strength, GPIO_PIN_TYPE_STD_WPU);

    Led(LEDALL_OFF);

}

/* InitializeOutput
* param:
* port..base port address e. g. GPIO_PORTJ_BASE
* pin..pin mask e. g. GPIO_PIN_0 | GPIO_PIN_4
*/
void InitializeOutput(uint32_t port, uint8_t pin)
{
    uint32_t strength;
    uint32_t pinType;

    /*configure pad as standard pin with default output current*/
    GPIOPadConfigGet(port, pin, &strength, &pinType);
    GPIOPadConfigSet(port, pin, strength, GPIO_PIN_TYPE_STD);
    /* set pin gpio port to output */
    GPIOPinTypeGPIOOutput(port, pin);

}


/* Led
* Switch on led 1 to led 4 due to parameter mask.
* param: mask output mask bit 0led 1, bit1 led1 etc.
*/
void Led(uint8_t mask)
{
    uint8_t valPort;

    valPort = 0;
    if (mask & LED1_ON) {
        valPort |= GPIO_PIN_1;
    }
    if (mask & LED2_ON)
    {
        valPort |= GPIO_PIN_0;
    }
    /* write value of val to pin 0 / 1 of port n */
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1 | GPIO_PIN_0, valPort);

    valPort = 0;
    if (mask & LED3_ON) {
        valPort |= GPIO_PIN_4;
    }
    if (mask & LED4_ON) {
        valPort |= GPIO_PIN_0;
    }
    /* write value of val to pin 0 / 4 of port f */
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0, valPort);
}


/* LedHandler
* Business logik of led task.
*/
void LedHandler(void)
{
    ++s_led_loop;
    s_led_loop %= s_led_count;
    Led(s_state[s_led_loop]);
}
