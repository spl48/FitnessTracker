/* FitnessTue9Group7
 *
 * display.c
 *
 *  Created on: 23/05/2020
 *      Author: Sean & Nick
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "utils/ustdlib.h"
#include "display.h"
#include "../OrbitOLED/OrbitOLEDInterface.h"

/*********************************************************
 * Initialise the Orbit OLED display
 *********************************************************/
void
initDisplay (void)
{
    OLEDInitialise ();
}

//*****************************************************************************
// Function to display a changing message on the Orbit OLED display.
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
    if (units != NULL)
    {
        int decimals = (num-intnum)*100;

        if (decimals < 10)
        { //Display a 0 in front of the decimal
            usnprintf(text_buffer, sizeof(text_buffer), "%s %s %1d.0%1d %s", str1, str2, intnum, decimals, units);

        }
        else
        {
            usnprintf(text_buffer, sizeof(text_buffer), "%s %s %1d.%2d %s", str1, str2, intnum, decimals, units);
        }
    }
    else
    {
        usnprintf(text_buffer, sizeof(text_buffer), "%s %s %3d %s", str1, str2, intnum, units);
    }

    // Update line on display.
    OLEDStringDraw (text_buffer, 0, charLine);
}
