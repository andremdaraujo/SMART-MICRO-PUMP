//
//	André A. M. Araújo
//	2022/01/24
//
//	This program is part of the Final Project presented to the Classpert Course
//	Making Embedded Systems by Elecia White.
//
//	The main objective is to control the flow of a micro pump using PID control.
//	The program has 3 modes of operation, MANUAL, AUTO and DEBUG.
//
//	MANUAL mode will read the voltage on a trimpot and adjust the PWM duty cycle
//	accordingly (0-3V input to 0-100% output).
//
//	AUTO mode will control the pump PWM using PID control with a flow sensor
//	reading for feedback. The trimpot controls the set point to the pump
//	(0-3V input to 85 - 500 mL/min).
//
//	TARGET:
//		STM32L152RB ("STM32L-Discovery" Board)
//
// 	INPUTS:
//		User Button:	PA0 	(Active high, rising and falling edges detection)
//		Pump Current:	PA3		(Micro pump electrical current)
//		Pump Flow:		PA4		(Micro pump flow)
//		Trimpot:		PA5		(Potentiometer to adjust PWM duty cycle/set point on MANUA/AUTO modes
//		UART RX:		PA10 	(Commands reception)
//
//	OUTPUTS:
//		UART TX:		PA9		(Data transmission)
//		DC Motor Drive:	PB6		(PWM)
//		Blue  LED:		PB6 	(PWM indication)
//		Green LED:		PB7 	(Mode indication)
//		Test output:	PC0		(Timing verification using an oscilloscope)
//
//	PERIPHERALS:
//		ADC:					Internal ADC (3 channels)
//		Sampling period timer:	TIM2 	(fS   = 100  Hz)
//		PWM generation:			TIM4 	(fPWM =  16 kHz)
//		Debounce timer:			TIM6 	(fDEB =   1 kHz)
//		UART:					USART1	(baud = 115200 bps, 8N1)
//
//	Developed in STM32CubeIDE 1.8.0
//

#include "main.h"		// Main program

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "adc.h"		// Internal ADC
#include "cli.h"		// Command Line Interface
#include "dma.h"		// Direct memory Access
#include "global.h"		// Global variables
#include "gpio.h"		// General Purpose Inputs and Outputs
#include "init.h"		// Initialization routines
#include "pid.h"		// PID controller
#include "pwm.h"		// PWM
#include "tim.h"		// Timers
#include "usart.h"		// UART for serial communication
#include "version.h"	// Version information

#define MIN_FLOW 100.0f	// Minimum flow the pump can achieve in this configuration
#define MAX_FLOW 500.0f	// Maximum flow the pump can achieve in this configuration

int main(void)
{
	sPID PID;

	uint16_t pulse = 0;

	volatile uint16_t ADC_counts[ADC_ACTIVE_CHANNELS];
	float ADC_voltages[ADC_ACTIVE_CHANNELS];

	uint16_t i = 0;
	uint8_t debug_counter = 0;

	float pump_current, pump_flow, pump_sqrt_flow;
	float trimpot;
	float MCU_temperature, 	MCU_voltage_ref;

	MCU_init();

	HAL_ADC_Start_DMA(&hadc, (uint32_t *)ADC_counts, ADC_ACTIVE_CHANNELS);
	HAL_TIM_Base_Start_IT(&htim2);				// Timer 2 for sampling period
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);	// Timer 4 for PWM generation

	PID_init(&PID);

	UART_TX_string("\n\r\n\r");
	UART_TX_string("Enter command, or press USER BUTTON on DISCO BOARD to start AUTO/MANUAL modes: \n\r");
	UART_TX_string(AVAILABLE_COMMANDS);
	UART_RX(rx_buffer);		// Starts serial reception

	while (op_mode == mode_idle)
	{
		// Wait for user command via console or button press
		if (flag_CRX != 0)
		{
			CLI_decode(rx_buffer);
		}
		if (debouncedButtonPressed != 0)
		{
			op_mode = mode_manual;
		}
	}

	while (1)
	{
		// Mode selection via user button (Manual/Auto)
		if (debouncedButtonPressed != 0)	// User button selects between
		{									// Manual and Auto modes
			if (op_mode != mode_auto)
			{
				op_mode = mode_auto;
				HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 1);
				UART_TX_string("\n\r\n\r");
				UART_TX_string("Auto mode selected. \n\r");
				UART_TX_string("Adjust pulse width via the trimpot on the DISCO BOARD. \n\r");
			}
			else
			{
				op_mode = mode_manual;
				HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 0);
				UART_TX_string("\n\r\n\r");
				UART_TX_string("Manual mode selected. \n\r");
				UART_TX_string("Adjust pulse width via the trimpot on the DISCO BOARD. \n\r");
			}
			debouncedButtonPressed = 0;
		}

		//	if (debouncedButtonReleased != 0)	// Debounced interrupts are also generated
		//	{									// when button is released
		//		// To do
		//		debouncedButtonReleased = 0;
		//	}

		if (flag_EOC != 0)	// Sampling time (dt) = 10ms (fS = 100 Hz)
		{
			for (i = 0; i < ADC_ACTIVE_CHANNELS; i++)
			{
				ADC_voltages[i] = ADC_counts[i] * ADC_V_REF / ADC_MAX_COUNTS;	// V = counts * 3.0 / (2^12 - 1)
			}
			HAL_GPIO_WritePin(OUT_TEST_GPIO_Port, OUT_TEST_Pin, 0);

			pump_current	= ADC_voltages[0] * 1000.0 / (1.0 * 6.6);	// mA (Ohm's law: I = V/R; R = 1 Ohm)
																		// AA filter gain: 6.6

			pump_sqrt_flow	= (ADC_voltages[1] - 0.5)  / 2.1;			// D6F-P0010A1 datasheet curve approximation
			pump_flow		= 1000 * pump_sqrt_flow * pump_sqrt_flow;	// mL/min

			trimpot 		= ADC_voltages[2];					// V

			MCU_temperature	= ADC_voltages[3];					// °C

			MCU_voltage_ref	= ADC_voltages[4];					// V

			flag_EOC = 0;
			flag_dt = 1;	// Will trigger the next PID Control iteration
		}

		if (flag_dt != 0)	// Sampling time (dt) = 10ms (fS = 100 Hz),
		{					// according to ADC EOC (ADC is trigger by Timer 2 interrupts)

			if (op_mode == mode_manual)			// Manual mode: PWM duty cycle is set based on
			{									// trimpot value read by the ADC

				HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 0);

				pulse = (uint16_t)(PWM_MAX_COUNTS * (trimpot / ADC_V_REF));

				PWM_setPulse(pulse);	// Updates duty cycle
			}
			else if (op_mode == mode_auto)
			{
				HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 1);

				// Set point update:
				// (Practical range for the pump: 85 to 500 mL/min)
				PID.set_point = MIN_FLOW + (trimpot / ADC_V_REF) * (MAX_FLOW - MIN_FLOW);

				// Feedback update
				PID.feedback = pump_flow;

				// Calculate control action
				PID_update(&PID);

				// Update pump drive level (PWM)
				pulse = (uint16_t)(PWM_MAX_COUNTS * PID.output);	// Converts Duty Cycle (%) to Pulse Width (timer counts)
				PWM_setPulse(pulse);								// Updates Duty Cycle

				// Send data (UART)
				UART_TX_string("SP:");
				UART_TX_float(PID.set_point);
				UART_TX_string("FB:");
				UART_TX_float(PID.feedback);
				UART_TX_string("E:");
				UART_TX_float(PID.error);

				UART_TX_string("P:");
				UART_TX_float(PID.proportional);
				UART_TX_string("I:");
				UART_TX_float(PID.integral);
				UART_TX_string("D:");
				UART_TX_float(PID.derivative);

				UART_TX_string("O:");
				UART_TX_float(PID.output);
				UART_TX_string("mA:");
				UART_TX_float(pump_current);

				UART_TX_string("°C:");
				UART_TX_float(MCU_temperature);
				UART_TX_string("VREF:");
				UART_TX_float(MCU_voltage_ref);

				UART_TX_string("\r ");
			}
			else if (op_mode == mode_debug)
			{
				if (flag_update_pulse != 0)
				{
					pulse = (uint16_t)(PWM_MAX_COUNTS * cli_input_value/100.0);	// Converts Duty Cycle (%) to Pulse Width (timer counts)
					PWM_setPulse(pulse);								// Updates Duty Cycle

					cli_input_value = 0;
					flag_update_pulse = 0;
				}

				debug_counter++;			// Software timer to blink the Green LED
				if (debug_counter >= 50)	// 500 ms on (1 Hz)
				{
					HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

					debug_counter = 0;
				}
			}
			flag_dt = 0;
		}

		if (flag_CRX != 0)
		{
			CLI_decode(rx_buffer);

			flag_CRX = 0;
		}

		if (flag_wrong_cmd != 0)
		{
			UART_TX_string("Wrong command! Available commands: \n\r ");
			UART_TX_string(AVAILABLE_COMMANDS);

			flag_wrong_cmd = 0;
		}
	}
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
	  // To do
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
	// To do
}
#endif /* USE_FULL_ASSERT */
