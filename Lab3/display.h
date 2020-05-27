#ifndef DISPLAY_H_
#define DISPLAY_H_

/* FitnessTue9Group7
 *
 * display.h
 *
 *  Created on: 23/05/2020
 *      Author: Sean & Nick
 */

#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
// Constants
//*****************************************************************************


// *******************************************************
// Initialise the Orbit OLED display
// *******************************************************
void
initDisplay (void);

//*****************************************************************************
// Function to display a changing message on the Orbit OLED display.
// The display has 4 rows of 16 characters, with 0, 0 at top left.
//*****************************************************************************
void
displayUpdate (char *str1, char *str2, double num, uint8_t charLine, char *units);

#endif /*DISPLAY_H_*/
