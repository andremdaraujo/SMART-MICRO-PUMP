#ifndef __GLOBAL_H
#define __GLOBAL_H

// Global Variables
extern volatile uint8_t  toggleGreenLED;			// Flags for toggling LEDs on main loop
extern volatile uint8_t  toggleBlueLED;			//

extern volatile uint8_t currentButton;				// Variables to store button states
extern volatile uint8_t previousButton;			//

extern volatile uint16_t debounceCounter;			// Variables for button debounce
extern volatile uint8_t  debouncedButtonPressed;	//
extern volatile uint8_t  debouncedButtonReleased;	//

#endif // __GLOBAL_H
