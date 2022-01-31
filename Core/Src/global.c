/*
 * global.c
 *
 *  Created on: Jan 28, 2022
 *      Author: Andre
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "global.h"

// Global Variables
volatile uint8_t  	toggleGreenLED = 0;			// Flags for toggling LEDs on main loop

volatile uint8_t 	currentButton  = 0;			// Variables to store button states
volatile uint8_t 	previousButton = 0;			//
volatile uint8_t 	debounceCounter = 0;		// Variables for button debounce
volatile uint8_t  	debouncedButtonPressed  = 0;//
volatile uint8_t  	debouncedButtonReleased = 0;//

volatile uint8_t	flag_dt = 0;				// Sampling period (dt) elapsed flag
volatile uint8_t	flag_EOC = 0;				// ADC End Of Conversion flag
