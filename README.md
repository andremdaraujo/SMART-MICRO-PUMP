# SMART-MICRO-PUMP

André A. M. Araújo
2022/02/21

This program is part of the Final Project presented to the Classpert Course
Making Embedded Systems by Elecia White.

The main objective is to control the flow of a micro pump using PID control.
The program has 3 modes of operation, MANUAL, AUTO and DEBUG.

MANUAL mode will read the voltage on a trimpot and adjust the PWM duty cycle
accordingly (0-3V input to 0-100% output).

AUTO mode will control the pump PWM using PID control with a flow sensor
reading for feedback. The trimpot controls the set point to the pump
(0-3V input to 85 - 500 mL/min).

TARGET:
	STM32L152RB ("STM32L-Discovery" Board)

INPUTS:
	User Button:	  PA0 	(Active high, rising and falling edges detection)
	Pump Current:	  PA3		(Micro pump electrical current)
	Pump Flow:		  PA4		(Micro pump flow)
	Trimpot:		    PA5		(Potentiometer to adjust PWM duty cycle/set point on MANUA/AUTO modes
	UART RX:		    PA10 	(Commands reception)

OUTPUTS:
	UART TX:		    PA9		(Data transmission)
	DC Motor Drive:	PB6		(PWM)
	Blue  LED:		  PB6 	(PWM indication)
	Green LED:		  PB7 	(Mode indication)
	Test output:	  PC0		(Timing verification using an oscilloscope)

PERIPHERALS:
	ADC:					          Internal ADC (3 channels)
	Sampling period timer:	TIM2 	(fS   = 100  Hz)
	PWM generation:		      TIM4 	(fPWM =  16 kHz)
	Debounce timer:		      TIM6 	(fDEB =   1 kHz)
	UART:					          USART1	(baud = 115200 bps, 8N1)

Developed in STM32CubeIDE 1.8.0
