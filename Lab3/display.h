#ifndef DISPLAY_H_
#define DISPLAY_H_

// *******************************************************
// display.h
//
// *******************************************************

#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
// Constants
//*****************************************************************************


// *******************************************************
// initButtons: Initialise the variables associated with the set of buttons
// defined by the constants above.
void
initDisplay (void);

// *******************************************************
// updateButtons: Function designed to be called regularly. It polls all
// buttons once and updates variables associated with the buttons if
// necessary.  It is efficient enough to be part of an ISR, e.g. from
// a SysTick interrupt.
void
displayUpdate (char *str1, char *str2, double num, uint8_t charLine, char *units);

#endif /*DISPLAY_H_*/
