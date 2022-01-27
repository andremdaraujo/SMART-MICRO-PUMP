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
//	accordingly (0-3 input to 0-100% output)
//
//	AUTO mode will control the pump PWM using the PID control and a flow sensor
//	reading for flow feedback.
//
//	TARGET:
//		STM32L152RB (STM32L-Discovery Board)
//
// 	INPUTS:
//		User Button:	PA0 	(Active high, rising and falling edges detection)
//		UART RX:		PA10 	(Commands reception)
//
//	OUTPUTS:
//		UART TX:		PA9		(Data transmission)
//		DC Motor:		PB6		(PWM)
//		Blue  LED:		PB6 	(PWM indication)
//		Green LED:		PB7 	(Mode indication)
//
//	PERIPHERALS:
//		ADC:					Intenral ADC (3 channels)
//		PWM generation:			TIM4 	(fPWM =  10 kHz)
//		Debounce timer:			TIM6 	(fDEB =   1 kHz)
//		Sampling period timer:	TIM7 	(fS   = 100  Hz)
//		UART:					USART1	(baud = 115200 bps, 8N1)
//
//	Developed in STM32CubeIDE 1.8.0
//

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

enum op_mode {manual_mode = 0, auto_mode = 1, debug_mode = 2};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEBOUNCE_STABLE_PERIOD 10				// Debounce period [ms]

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Global Variables
volatile uint8_t  toggleGreenLED = 0;			// Flags for toggling LEDs on main loop
volatile uint8_t  toggleBlueLED  = 0;			//

volatile uint8_t currentButton  = 0;			// Variables to store button states
volatile uint8_t previousButton = 0;			//

volatile uint16_t debounceCounter = 0;			// Variables for button debounce
volatile uint8_t  debouncedButtonPressed  = 0;	//
volatile uint8_t  debouncedButtonReleased = 0;	//

register uint32_t* reg_SP __asm("sp");			// Stack Pointer

char uartTXbuf[50];								// UART buffer for TX data

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
	uint32_t test_var = 1024;
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

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);	// Timer 4 for PWM generation
	HAL_TIM_Base_Start_IT(&htim7);				// Timer 7 for sampling period

	sprintf(uartTXbuf, "MES Exercise 8: \n");
	HAL_UART_Transmit(&huart1, uartTXbuf, strlen(uartTXbuf), HAL_MAX_DELAY);

	sprintf(uartTXbuf,
			"Global var 'toggleGreenLED' address: 0x%08X \n",
			&toggleGreenLED);
	HAL_UART_Transmit(&huart1, uartTXbuf, strlen(uartTXbuf), HAL_MAX_DELAY);

	sprintf(uartTXbuf,
			"Stack Pointer Register 'reg_SP' value: 0x%08X \n",
			((uint32_t)reg_SP));
	HAL_UART_Transmit(&huart1, uartTXbuf, strlen(uartTXbuf), HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
		HAL_Delay(500);
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

