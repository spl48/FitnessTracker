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

#define BUF_SIZE 10
#define SAMPLE_RATE_HZ 10

#define SYSTICK_RATE_HZ 100
#define ACCTICK_RATE_HZ 2

#define STEP_THRESHOLD 1500
#define STEP_LENGTH 0.0009

#define MILES_CONSTANT 0.6213712

#define TEST_MODE_STEPS_UP 100
#define TEST_MODE_DIST_UP 0.09
#define TEST_MODE_STEPS_DOWN 500
#define TEST_MODE_DIST_DOWN 0.45

#define HOLD_THRESHOLD 100000

typedef struct{
    int16_t x;
    int16_t y;
    int16_t z;
} vector3_t;

//*****************************************************************************
// Global variables
//*****************************************************************************
static circBuf_t g_xinBuffer;        // Buffer of size BUF_SIZE integers (sample values)
static circBuf_t g_yinBuffer;
static circBuf_t g_zinBuffer;
static uint32_t g_ulSampCnt;    // Counter for the interrupts
volatile uint8_t accTick = false;
static uint32_t steps = 0;
static double distance = 0;
static  uint8_t wasBelow = true;
static uint8_t distanceToggle = false;
static uint8_t unitsToggle = false;
static uint8_t testMode = false;
static uint32_t holdCounter = 0;

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
 * Initialisation functions: clock, SysTick, PWM
 ***********************************************************
 * Clock
 ***********************************************************/
void
initClock (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
}

/*********************************************************
 * initDisplay
 *********************************************************/
void
initDisplay (void)
{
    // Initialise the Orbit OLED display
    OLEDInitialise ();
}

//*****************************************************************************
// Function to display a changing message on the display.
// The display has 4 rows of 16 characters, with 0, 0 at top left.
//*****************************************************************************
void
displayUpdate (char *str1, char *str2, double num, uint8_t charLine, char *units)
{
    char text_buffer[17];           //Display fits 16 characters wide.

    int intnum = num;


    // "Undraw" the previous contents of the line to be updated.
    OLEDStringDraw ("                ", 0, charLine);
    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    if (units != NULL) {
        int decimals = (num-intnum)*100;
        usnprintf(text_buffer, sizeof(text_buffer), "%s %s %1d.%2d %s", str1, str2, intnum, decimals, units);
    } else {
        usnprintf(text_buffer, sizeof(text_buffer), "%s %s %3d %s", str1, str2, intnum, units);
    }
    // Update line on display.
    OLEDStringDraw (text_buffer, 0, charLine);
}

/*********************************************************
 * initAccl
 *********************************************************/
void
initAccl (void)
{
    char    toAccl[] = {0, 0};  // parameter, value

    /*
     * Enable I2C Peripheral
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    /*
     * Set I2C GPIO pins
     */
    GPIOPinTypeI2C(I2CSDAPort, I2CSDA_PIN);
    GPIOPinTypeI2CSCL(I2CSCLPort, I2CSCL_PIN);
    GPIOPinConfigure(I2CSCL);
    GPIOPinConfigure(I2CSDA);

    /*
     * Setup I2C
     */
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

    GPIOPinTypeGPIOInput(ACCL_INT2Port, ACCL_INT2);

    //Initialize ADXL345 Acceleromter

    // set +-2g, 13 bit resolution, active low interrupts
    toAccl[0] = ACCL_DATA_FORMAT;
    toAccl[1] = (ACCL_FULL_RES | ACCL_FULL_RES);
    I2CGenTransmit(toAccl, 1, WRITE, ACCL_ADDR);

    toAccl[0] = ACCL_PWR_CTL;
    toAccl[1] = ACCL_MEASURE;
    I2CGenTransmit(toAccl, 1, WRITE, ACCL_ADDR);


    toAccl[0] = ACCL_BW_RATE;
    toAccl[1] = ACCL_RATE_100HZ;
    I2CGenTransmit(toAccl, 1, WRITE, ACCL_ADDR);

    toAccl[0] = ACCL_INT;
    toAccl[1] = 0x00;       // Disable interrupts from accelerometer.
    I2CGenTransmit(toAccl, 1, WRITE, ACCL_ADDR);

    toAccl[0] = ACCL_OFFSET_X;
    toAccl[1] = 0x00;
    I2CGenTransmit(toAccl, 1, WRITE, ACCL_ADDR);

    toAccl[0] = ACCL_OFFSET_Y;
    toAccl[1] = 0x00;
    I2CGenTransmit(toAccl, 1, WRITE, ACCL_ADDR);

    toAccl[0] = ACCL_OFFSET_Z;
    toAccl[1] = 0x00;
    I2CGenTransmit(toAccl, 1, WRITE, ACCL_ADDR);
}

/********************************************************
 * Function to read accelerometer
 ********************************************************/
vector3_t
getAcclData (void)
{
    char    fromAccl[] = {0, 0, 0, 0, 0, 0, 0}; // starting address, placeholders for data to be read.
    vector3_t acceleration;
    uint8_t bytesToRead = 6;

    fromAccl[0] = ACCL_DATA_X0;
    I2CGenTransmit(fromAccl, bytesToRead, READ, ACCL_ADDR);

    acceleration.x = (fromAccl[2] << 8) | fromAccl[1]; // Return 16-bit acceleration readings.
    acceleration.y = (fromAccl[4] << 8) | fromAccl[3];
    acceleration.z = (fromAccl[6] << 8) | fromAccl[5];

    return acceleration;
}

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
 * main
 ********************************************************/
int
main (void)
{
    //Code for enabling Orbit Booster Switch 1

    //**********************

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOInput (GPIO_PORTA_BASE, GPIO_PIN_7);
    GPIOPadConfigSet (GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA,
           GPIO_PIN_TYPE_STD_WPD);


    //**********************


    //Enabling floating point calculations
    FPUEnable();

    //**********************

    vector3_t acceleration_raw;
    vector3_t refOrientation;


    initClock ();
    initAccl ();
    initDisplay ();
    initButtons ();
    initCircBuf (&g_xinBuffer, BUF_SIZE);
    initCircBuf (&g_yinBuffer, BUF_SIZE);
    initCircBuf (&g_zinBuffer, BUF_SIZE);
    initSysTick ();

    OLEDStringDraw ("Total Steps", 0, 0);

    acceleration_raw = getAcclData();
    writeCircBuf (&g_xinBuffer, acceleration_raw.x);
    writeCircBuf (&g_yinBuffer, acceleration_raw.y);
    writeCircBuf (&g_zinBuffer, acceleration_raw.z);
    refOrientation = calculateMeanBuffer();
    char* units = "km";

    while (1)
    {

        //Code for reading the switch state

        //**************************
        bool switchState = (GPIOPinRead (GPIO_PORTA_BASE, GPIO_PIN_7) == GPIO_PIN_7);
        if(switchState){
            testMode = true;
        }else{
            testMode = false;
        }
        switchState = false;

        //**************************

        /*Code for held down button function using guidelines from
        Project Demonstration Guidelines and Timetable 13 05 2020

        */
        //**************************

        bool heldState = (GPIOPinRead (GPIO_PORTD_BASE, GPIO_PIN_2) == GPIO_PIN_2);
        if(heldState){
            holdCounter++;
        }else{
            holdCounter = 0;
        }

        if(holdCounter > HOLD_THRESHOLD){

            distance = 0;
            steps = 0;

        }
        heldState = false;


        //**************************

        uint8_t butState;
        updateButtons ();
        butState = checkButton (UP);
        switch (butState)
        {
        case RELEASED:
            if(testMode){
                steps += TEST_MODE_STEPS_UP;
                distance += TEST_MODE_DIST_UP;
            }else
            {
            if(distanceToggle)
            {
            unitsToggle = !unitsToggle;
            if(unitsToggle)
            {
                units = "mi.";
            }
            else
            {
                units = "km.";
            }
            }
            break;
            }
        }
        butState = checkButton (DOWN);
        switch (butState)
        {
        case RELEASED:
            if(testMode){
                            steps -= TEST_MODE_STEPS_DOWN;
                            distance -= TEST_MODE_DIST_DOWN;
                        }
            break;
        }

        butState = checkButton (LEFT);
                switch (butState)
                {
                case RELEASED:
                    distanceToggle = !distanceToggle;
                    break;
                }
        butState = checkButton (RIGHT);
                switch (butState)
                {
                case RELEASED:
                distanceToggle = !distanceToggle;
                   break;
                       }

        if (accTick)
        {
            accTick = false;
            acceleration_raw = getAcclData();

            writeCircBuf (&g_xinBuffer, acceleration_raw.x);
            writeCircBuf (&g_yinBuffer, acceleration_raw.y);
            writeCircBuf (&g_zinBuffer, acceleration_raw.z);

            vector3_t meanBuffer = calculateMeanBuffer();

            if(!distanceToggle){
                displayUpdate("Num.", "Steps", steps, 0, NULL);
            }
            else{
                if(unitsToggle){


                displayUpdate("Dist.", "", distance*MILES_CONSTANT, 0, units);

                }

                else

                {

                 displayUpdate("Dist.", "", distance, 0, units);

                }

            }

            //Calculation for total acceleration which then gets casted into a 16bit int
            //******************

            int16_t accelerationTotal = (int16_t) sqrt((acceleration_raw.x * 4)*(acceleration_raw.x * 4)+
                                                 (acceleration_raw.y * 4)*(acceleration_raw.y * 4)+
                                                 (acceleration_raw.z * 4)*(acceleration_raw.z * 4));

            if((accelerationTotal > STEP_THRESHOLD) && (wasBelow)){
                wasBelow = false;
                steps++;
                distance += STEP_LENGTH;
            }

            if((accelerationTotal < STEP_THRESHOLD) && (!wasBelow)){
                wasBelow = true;
            }

            //******************

        }

    }
}
