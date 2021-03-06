/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Defined MACROS
//Uncomment those MACROS below for program to execute ONLY
//#define BLINK_LED
//#define ADC_VREFINT
#define ADC_TEMPSENSOR
//#define TOGGLE

#define TS_CAL1_TEMP 30 //FROM DATASHEET
#define TS_CAL1 *((uint16_t*) 0x1FFF75A8) //FROM DATASHEET
#define TS_CAL2_TEMP 130 //FROM DATASHEET
#define TS_CAL2 *((uint16_t*) 0x1FFF75CA) //FROM DATASHEET
#define SCALE 1.1 //temp is calculated at 3.0V so needs to be scaled (3.3/3.0)
#define VREFINT_CAL *((uint16_t*) 0x1FFF75AA)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
ADC_HandleTypeDef hadc1;
ADC_ChannelConfTypeDef sConfig = {0};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  uint32_t data;
  HAL_ADC_Init(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_Delay(1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  If either of MACROS are defined then do the following initilization
#if defined(BLINK_LED) || defined(TOGGLE)
  int changeState = 0;
  float VRefInt = 0;
  float VRef = 0;
  uint16_t tempRaw;
  float tempData_1;
  float tempData_2;
//  Define common sConfig states for both channels to avoid redundancy
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
#endif

#ifdef ADC_VREFINT
  float VRefInt = 0;
  float VRef = 0;
//  Initialize the config channels for Vrefint beforehand
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5 ;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
//  Set the channel
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
//  Validate the channel
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
#endif

#ifdef ADC_TEMPSENSOR
  uint16_t tempRaw;
  float tempData_1;
  float tempData_2;
//  Initiazlie the config channel for temperature sensor beforehand
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
//  Set the channel
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
//  Validate the channel
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

#endif

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#ifdef BLINK_LED //If this is defined then execute blinking led
	  if(!HAL_GPIO_ReadPin(BUTTON_BLUE_GPIO_Port, BUTTON_BLUE_Pin)){
			  if(!changeState){ //prevents toggling if button is on hold
				  HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
				  changeState = 1;
			  }
	  } else { //this is only active if the button is not pressed. Allows for toggling
		  changeState = 0;
	  }
#endif


#ifdef ADC_VREFINT //If this is defined then execute Vrefint
//	  	  Start and poll until the channel is ready
	      ADC_Enable(&hadc1);
	  	  HAL_ADC_Start(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

//		  Get internal voltage reference and voltage reference
		  data = HAL_ADC_GetValue(&hadc1);
		  VRefInt = data*3.3/4095*SCALE;
		  VRef = 3*(float)VREFINT_CAL/(data*SCALE);

//		  Stop the channel once ready
		  HAL_ADC_Stop(&hadc1);
#endif

#ifdef ADC_TEMPSENSOR //If this is defined then execute temperature sensor
//		  Start and poll to get the channel ready
		  ADC_Enable(&hadc1);
		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

//		  Get Temperature data
		  data = HAL_ADC_GetValue(&hadc1);
		  tempRaw = data;

//		  Used both internal method of the HAL class and done manual calculation
//		  Used to verify that both data are close together for accuracy
		  tempData_1 = __HAL_ADC_CALC_TEMPERATURE(3300, tempRaw, ADC_RESOLUTION_12B);
		  tempData_2 = (float) (TS_CAL2_TEMP - TS_CAL1_TEMP)/(TS_CAL2 - TS_CAL1) *
				  ((tempRaw * SCALE) - TS_CAL1) + 30;

//		  Stop the channel once done
		  HAL_ADC_Stop(&hadc1);
		  ADC_Disable(&hadc1);

#endif

#ifdef TOGGLE //If this is defined then execute Toggle
		  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
		  HAL_Delay(1);
//		  Check if blue button has been pressed
		  if(!HAL_GPIO_ReadPin(BUTTON_BLUE_GPIO_Port, BUTTON_BLUE_Pin)){
				  if(!changeState){ //prevents toggling if button is on hold
					  HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
					  changeState = 1;
				  }
		  } else { //this is only active if the button is not pressed. Allows for toggling
			  changeState = 0;
		  }

//		  Check if LED is lit
		  if (HAL_GPIO_ReadPin(LED_GREEN_GPIO_Port, LED_GREEN_Pin)){
//			  Get Vref Data if GREEN led is HIGH
//			  Config channel for Vref here
//			  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
//			  HAL_Delay(1);

			  sConfig.Channel = ADC_CHANNEL_VREFINT;
//			  Set the channel here
			  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

//			  Validate the channel
			  if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
//			  Start the ADC channel and poll until the channel is ready
			  ADC_Enable(&hadc1);
			  HAL_ADC_Start(&hadc1);
			  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

//			  Get internal voltage reference and voltage reference
			  data = HAL_ADC_GetValue(&hadc1);
			  VRefInt = data*3.3/4096*SCALE;
			  VRef = 3*(float)VREFINT_CAL/(data*SCALE); //Formula from datasheet

//			  Stop the channel once done
			  HAL_ADC_Stop(&hadc1);
			  ADC_Disable(&hadc1);
		  } else {
//			  Get Temperature sensor Data if GREEN led is LOW
//			  Config channel for temperature sensor
//			  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
//			  HAL_Delay(1);

			  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;

//			  Set the channel here
			  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

//			  Validate the channel here
			  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

//			  Start the ADC channel and pol until the channel is ready
			  ADC_Enable(&hadc1);
			  HAL_ADC_Start(&hadc1);
			  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

//			  Get temperature sensor readings
			  data = HAL_ADC_GetValue(&hadc1);
			  tempRaw = data;

//			  Used both internal method of the HAL class and done manual calculation
//			  Used to verify that both data are close together for accuracy
			  tempData_1 = __HAL_ADC_CALC_TEMPERATURE(3300, tempRaw, ADC_RESOLUTION_12B);
			  tempData_2 = (float) (TS_CAL2_TEMP - TS_CAL1_TEMP)/(TS_CAL2 - TS_CAL1) *
			  				  ((tempRaw * SCALE) - TS_CAL1) + 30;
//			  tempData_2 = ((3.3*ADC_VAL[2]/4095 - V25)/Avg_Slope)+25;

//			  Stop the channel once done
			  HAL_ADC_Stop(&hadc1);
			  ADC_Disable(&hadc1);
		  }
#endif

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 40;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_BLUE_Pin */
  GPIO_InitStruct.Pin = BUTTON_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_BLUE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
