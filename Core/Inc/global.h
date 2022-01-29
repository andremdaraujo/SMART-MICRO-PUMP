#ifndef __GLOBAL_H
#define __GLOBAL_H

// Homework 8
extern volatile uint32_t uninit_var;
extern volatile uint32_t init_var;

// Global Variables
extern volatile uint8_t  toggleGreenLED;

extern volatile uint8_t currentButton;
extern volatile uint8_t previousButton;
extern volatile uint8_t debounceCounter;
extern volatile uint8_t debouncedButtonPressed;
extern volatile uint8_t debouncedButtonReleased;

extern volatile uint8_t	flag_dt;



#endif // __GLOBAL_H
