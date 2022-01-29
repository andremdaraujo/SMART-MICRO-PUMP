#ifndef __GLOBAL_H
#define __GLOBAL_H

// Global Variables
extern volatile uint8_t  toggleGreenLED;

extern volatile uint8_t currentButton;
extern volatile uint8_t previousButton;
extern volatile uint8_t debounceCounter;
extern volatile uint8_t debouncedButtonPressed;
extern volatile uint8_t debouncedButtonReleased;

extern volatile uint8_t	flag_dt;

#endif // __GLOBAL_H
