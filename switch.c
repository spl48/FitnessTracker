/* FitnessTue9Group7
 *
 * switch.c
 *
 *  Created on: 29/05/2020
 *      Author: Sean & Nick
 */
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "inc/tm4c123gh6pm.h"

void
initSwitch (void){
        //Code for enabling Orbit Booster Switch 1
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        GPIOPinTypeGPIOInput (GPIO_PORTA_BASE, GPIO_PIN_7);
        GPIOPadConfigSet (GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
};
