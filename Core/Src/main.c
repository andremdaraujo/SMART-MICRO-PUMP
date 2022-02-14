/* USER CODE BEGIN Header */

//
//	André A. M. Araújo
//	2022/01/24
//
//	This program is part of the Final Project presented to the Classpert Course
//	Making Embedded Systems by Elecia White.
//
//	The main objective is to control the flow of a micro pump using PID control.
//	The program has 2 modes of operation, MANUAL and AUTO.
//
//	MANUAL mode will read the voltage on a trimpot and adjust the PWM duty cycle
//	accordingly (0-3V input to 0-100% output).
//
//	AUTO mode will control the pump PWM using PID control with a flow sensor
//	reading as feedback.
//
//	TARGET:
//		STM32L152RB ("STM32L-Discovery" Board)
//
// 	INPUTS:
//		User Button:	PA0 	(Active high, rising and falling edges detection)
//		Pump Current:	PA3		(Micro pump electrical current)
//		Pump Flow:		PA4		(Micro pump flow)
//		Trimpot:		PA5		(Potentiometer to adjust PWM duty cycle on MANUAL MODE)
//		UART RX:		PA10 	(Commands reception)
//
//
//	OUTPUTS:
//		UART TX:		PA9		(Data transmission)
//		DC Motor:		PB6		(PWM)
//		Blue  LED:		PB6 	(PWM indication)
//		Green LED:		PB7 	(Mode indication)
//		Test output:	PC0		(Timing verification using an oscilloscope)
//
//	PERIPHERALS:
//		ADC:					Internal ADC (3 channels)
//		Sampling period timer:	TIM2 	(fS   = 100  Hz)
//		PWM generation:			TIM4 	(fPWM =  10 kHz)
//		Debounce timer:			TIM6 	(fDEB =   1 kHz)
//		UART:					USART1	(baud = 115200 bps, 8N1)
//
//	Developed in STM32CubeIDE 1.8.0
//

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "global.h"
#include "pid.h"
#include "pwm.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

enum operation_mode {mode_manual = 0, mode_auto = 1, mode_debug = 2};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define FLOW_RANGE 500.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void SystemClock_Config(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	enum operation_mode op_mode = mode_manual;
	sPID PID;

	uint16_t pulse = 0;

	volatile uint16_t ADC_counts[ADC_ACTIVE_CHANNELS];
	float ADC_voltages[ADC_ACTIVE_CHANNELS];

	uint16_t i = 0;

	float pump_current, pump_flow, pump_sqrt_flow;
	float trimpot;
	float MCU_temperature, 	MCU_voltage_ref;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_ADC_Init();
//  MX_TIM4_Init();
//  MX_TIM6_Init();
//  MX_DMA_Init();
//  MX_USART1_UART_Init();
//  MX_TIM2_Init();
//  /* USER CODE BEGIN 2 */

	MX_GPIO_Init();
	MX_TIM2_Init();
	MX_TIM4_Init();
	MX_TIM6_Init();
	MX_DMA_Init();
	MX_ADC_Init();
	MX_USART1_UART_Init();

	HAL_ADC_Start_DMA(&hadc, (uint32_t *)ADC_counts, ADC_ACTIVE_CHANNELS);

	HAL_TIM_Base_Start_IT(&htim2);				// Timer 2 for sampling period
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);	// Timer 4 for PWM generation

	UART_TX_string("Smart Micro Pump\n");
	UART_RX(rx_buffer);

	PID_init(&PID);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		// Mode selection via user button (Manual/Auto)
		if (debouncedButtonPressed != 0)	// User button selects between
		{									// Manual and Auto modes
			if (op_mode == mode_manual)
			{
				op_mode = mode_auto;
				HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 1);
			}
			else if (op_mode == mode_auto)
			{
				op_mode = mode_manual;
				HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 0);
			}
			debouncedButtonPressed = 0;
		}

		//	if (debouncedButtonReleased != 0)	// Interrupts are also generated when
		//	{									// button is released
		//		debouncedButtonReleased = 0;
		//	}

		if (flag_EOC != 0)	// Sampling time (dt) = 10ms (fS = 100 Hz)
		{
			flag_EOC = 0;

			for (i = 0; i < ADC_ACTIVE_CHANNELS; i++)
			{
				ADC_voltages[i] = ADC_counts[i] * ADC_V_REF / ADC_MAX_COUNTS;
			}
			HAL_GPIO_WritePin(OUT_TEST_GPIO_Port, OUT_TEST_Pin, 0);

			pump_current	= ADC_voltages[0] * 1000.0 / (1.0 * 5.89);	// mA (Ohm's law: I = V/R; R = 1 Ohm)
																		// AA filter gain: 6.6

			pump_sqrt_flow	= (ADC_voltages[1] - 0.5)  / 2.1;			// D6F-P0010A1 datasheet curve approximation
			pump_flow		= 1000 * pump_sqrt_flow * pump_sqrt_flow;	// mL/min

			trimpot 		= ADC_voltages[2];					// V

			MCU_temperature	= ADC_voltages[3];					// °C

			MCU_voltage_ref	= ADC_voltages[4];					// V

			flag_dt = 1;	// Will trigger the next PID Control iteration
		}

		if (flag_dt != 0)	// Sampling time (dt) = 10ms (fS = 100 Hz),
		{					// according to ADC EOC (which is trigger by Timer 2 interrupts)

			if (op_mode == mode_manual)			// Manual mode: PWM duty cycle is set based on
			{									// trimpot value read by the ADC

				pulse = (uint16_t)(PWM_MAX_COUNTS * (trimpot / ADC_V_REF));

				sprintf(tx_buffer, "PWM pulse: %4d \n", pulse);
				UART_TX_string(tx_buffer);

				PWM_setPulse(pulse);	// Updates duty cycle
			}
			else if (op_mode == mode_auto)
			{
				// Set point update
				PID.set_point = 85 + (trimpot / ADC_V_REF) * (FLOW_RANGE - 85);	// Practical range for the pump (85 to 500 mL/min)

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

				UART_TX_string("OUT:");
				UART_TX_float(PID.output);
				UART_TX_string("mA:");
				UART_TX_float(pump_current);

				UART_TX_string("\r ");
			}
			flag_dt = 0;
		}

		if (op_mode == mode_debug)
		{
			// To do
		}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

