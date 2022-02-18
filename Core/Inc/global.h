#ifndef __GLOBAL_H
#define __GLOBAL_H

enum operation_mode {mode_idle = 0, mode_manual = 1, mode_auto = 2, mode_debug = 3};

// Global Variables
extern volatile uint8_t  toggleGreenLED;

extern volatile uint8_t currentButton;
extern volatile uint8_t previousButton;
extern volatile uint8_t debounceCounter;
extern volatile uint8_t debouncedButtonPressed;
extern volatile uint8_t debouncedButtonReleased;

extern volatile uint8_t	flag_dt;
extern volatile uint8_t	flag_EOC;
extern volatile uint8_t	flag_CRX;
extern volatile uint8_t	flag_wrong_cmd;
extern volatile uint8_t	flag_update_pulse;

extern enum operation_mode op_mode;

#endif // __GLOBAL_H
