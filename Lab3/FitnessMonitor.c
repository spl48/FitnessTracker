/* FitnessTue9Group7
 *
 * FitnessMonitor.c
 *
 *  Created on: 15/03/2020
 *      Author: Sean & Nick
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "../OrbitOLED/OrbitOLEDInterface.h"
#include "utils/ustdlib.h"
#include "acc.h"
#include "i2c_driver.h"
#include "circBufT.h"
#include "buttons4.h"
#include "driverlib/fpu.h"
#include "display.h"
#include "accelerometer.h"

#define BUF_SIZE 10
#define SAMPLE_RATE_HZ 10

#define SYSTICK_RATE_HZ 100
#define ACCTICK_RATE_HZ 2

#define STEP_THRESHOLD 1500
#define STEP_LENGTH 0.0009

#define MILES_CONSTANT 0.6213712

#define TEST_MODE_STEPS_UP 100
#define TEST_MODE_STEPS_DOWN 500

#define HOLD_THRESHOLD 100000

typedef struct{
    int16_t x;
    int16_t y;
    int16_t z;
} vector3_t;


//*****************************************************************************
// Global variables
//*****************************************************************************
static circBuf_t g_xinBuffer;
static circBuf_t g_yinBuffer;
static circBuf_t g_zinBuffer;
volatile uint8_t accTick = false;
static uint32_t steps = 0;
static  uint8_t wasBelow = true;
static uint8_t distanceScreen = false; //true when board is displaying distance screen
static uint8_t testMode = false;
static uint32_t holdCounter = 0;
static uint8_t unitsToggle = false; //false when kilometers, true when miles
static char units[2][10] = {"km.", "mi."};  //Array of the units that can be displayed


/*******************************************
 *      Local prototypes
 *******************************************/
void initClock (void);
void initDisplay (void);
void displayUpdate (char *str1, char *str2, double num, uint8_t charLine, char *units);
void initAccl (void);
vector3_t getAcclData (void);
void SysTickIntHandler (void);


//*******************************************************************
//
// The interrupt handler for the SysTick interrupt.
//
//*******************************************************************
void
SysTickIntHandler (void)
{
    static uint8_t accTickCount = 0;
    const uint8_t ticksPerSlow = SYSTICK_RATE_HZ / ACCTICK_RATE_HZ;

    if (++accTickCount >= ticksPerSlow)
    {                       // Signal a slow tick
        accTickCount = 0;
        accTick = true;
    }

}


//*******************************************************************
//
// Initialise the SysTick timer
//
//*******************************************************************
void
initSysTick (void)
{
    //
    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
    SysTickPeriodSet (SysCtlClockGet () / SYSTICK_RATE_HZ);
    //
    // Register the interrupt handler
    SysTickIntRegister (SysTickIntHandler);
    //
    // Enable interrupt and device
    SysTickIntEnable ();
    SysTickEnable ();
}


/***********************************************************
 * Initialise the clock
 ***********************************************************/
void
initClock (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
}


//*******************************************************************
//
// Calculates the mean of the x,y, and z buffer and returns a 3 space vector vector3_t of the results
//
//*******************************************************************
vector3_t
calculateMeanBuffer ()
{
    uint16_t i;
    vector3_t meanVector;
    uint32_t sumx, sumy, sumz = 0;
    for (i = 0; i < BUF_SIZE; i++)
    {
        sumx = sumx + readCircBuf (&g_xinBuffer);
        sumy = sumy + readCircBuf (&g_yinBuffer);
        sumz = sumz + readCircBuf (&g_zinBuffer);
    }

    meanVector.x = ((2 * sumx + BUF_SIZE) / 2 / BUF_SIZE);
    meanVector.y = ((2 * sumy + BUF_SIZE) / 2 / BUF_SIZE);
    meanVector.z = ((2 * sumz + BUF_SIZE) / 2 / BUF_SIZE);

    return meanVector;
}


/********************************************************
 * Function to run all initialisation functions
 ********************************************************/
void
initialiseAll (void)
{
    initClock ();
    initAccl ();
    initDisplay ();
    initButtons ();
    initCircBuf (&g_xinBuffer, BUF_SIZE);
    initCircBuf (&g_yinBuffer, BUF_SIZE);
    initCircBuf (&g_zinBuffer, BUF_SIZE);
    initSysTick ();
}


/********************************************************
 * Checks SW1 and sets testMode true if it is switched up to indicate the board is in test mode.
 * Sets false if not switched up to indicate board in standard use mode
 ********************************************************/
void
checkBoardMode(void)
{
    bool switchState = (GPIOPinRead (GPIO_PORTA_BASE, GPIO_PIN_7) == GPIO_PIN_7);
    if(switchState)
    {
        testMode = true;
    }
    else
    {
        testMode = false;
    }

    switchState = false;
}


/********************************************************
 * Checks the reset button is being held down. If so adds to the holdCounter. If the counter exceeds the threshold, steps are reset
 ********************************************************/
void
checkResetButton(void)
{

    /*Code for held down button function using guidelines from
    Project Demonstration Guidelines and Timetable 13 05 2020

    */

    bool heldState = (GPIOPinRead (GPIO_PORTD_BASE, GPIO_PIN_2) == GPIO_PIN_2);
    if(heldState)
    {
        holdCounter++;
    }
    else
    {
        holdCounter = 0;
    }

    if(holdCounter > HOLD_THRESHOLD)
    {
        steps = 0;
    }

    heldState = false;
}


/**************************
*Check the buttons and carries out the corresponding actions if a button is pushed
**************************/
void
checkOtherButtons(void)
{
    uint8_t butState;
    updateButtons ();

    butState = checkButton (UP);
    switch (butState)
    {
        case RELEASED:
            if(testMode)
            {
                steps += TEST_MODE_STEPS_UP;
            }
            else if (distanceScreen)
            {
                unitsToggle = !unitsToggle;
            }

            break;
    }

    butState = checkButton (DOWN);
    switch (butState)
    {
        case RELEASED:
            if(testMode)
            {
                if(steps < TEST_MODE_STEPS_DOWN)
                {
                    steps = 0;
                }
                else
                {
                    steps -= TEST_MODE_STEPS_DOWN;
                }

            break;
            }
    }

    butState = checkButton (LEFT);
    switch (butState)
    {
        case RELEASED:
            distanceScreen = !distanceScreen;
            break;
    }

    butState = checkButton (RIGHT);
    switch (butState)
    {
        case RELEASED:
        distanceScreen = !distanceScreen;
        break;
    }
}

/**************************
* Refreshes the display to show the correct and updated information
**************************/
void
refreshDisplay(void)
{
    if(!distanceScreen)
    {
        displayUpdate("Num.", "Steps", steps, 0, NULL);
    }
    else
    {
        if(unitsToggle) //Miles
        {
            displayUpdate("Dist.", "", steps*STEP_LENGTH*MILES_CONSTANT, 0, units[unitsToggle]);
        }
        else //Kilometers
        {
            displayUpdate("Dist.", "", steps*STEP_LENGTH, 0, units[unitsToggle]);
        }
    }
}


/**************************
* Writes the new acceleration values to the circular buffer
**************************/
void
updateBuffer(vector3_t acceleration_raw)
{
    writeCircBuf (&g_xinBuffer, acceleration_raw.x);
    writeCircBuf (&g_yinBuffer, acceleration_raw.y);
    writeCircBuf (&g_zinBuffer, acceleration_raw.z);

    vector3_t meanBuffer = calculateMeanBuffer();
}


/**************************
* Checks if the norm of acceleration exceeds 1.5g and was below 1.5g at last check. If so adds a step.
**************************/
void
checkIfStep(vector3_t acceleration_raw)
{
    //******************
    //Calculation for total acceleration which then gets casted into a 16bit int
    //******************

    int16_t accelerationTotal = (int16_t) sqrt((acceleration_raw.x * 4)*(acceleration_raw.x * 4)+
                                         (acceleration_raw.y * 4)*(acceleration_raw.y * 4)+
                                         (acceleration_raw.z * 4)*(acceleration_raw.z * 4));

    if((accelerationTotal > STEP_THRESHOLD) && (wasBelow))
    {
        wasBelow = false;
        steps++;
    }

    if((accelerationTotal < STEP_THRESHOLD) && (!wasBelow))
    {
        wasBelow = true;
    }
}


/********************************************************
 * Main
 ********************************************************/
int
main (void)
{
    //Code for enabling Orbit Booster Switch 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOInput (GPIO_PORTA_BASE, GPIO_PIN_7);
    GPIOPadConfigSet (GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
    FPUEnable();  //Enabling floating point calculations

    vector3_t acceleration_raw;
    initialiseAll();
    displayUpdate("Num.", "Steps", steps, 0, NULL);
    acceleration_raw = getAcclData();
    writeCircBuf (&g_xinBuffer, acceleration_raw.x);
    writeCircBuf (&g_yinBuffer, acceleration_raw.y);
    writeCircBuf (&g_zinBuffer, acceleration_raw.z);

    // MAIN LOOP
    while (1)
    {
        checkBoardMode();

        checkResetButton();

        checkOtherButtons();

        //**************************
        // If tick counter then check accelerometer for a step and update display
        //**************************
        if (accTick)
        {
            accTick = false;
            acceleration_raw = getAcclData();

            updateBuffer(acceleration_raw);

            refreshDisplay();

            checkIfStep(acceleration_raw);
        }

    }
}
