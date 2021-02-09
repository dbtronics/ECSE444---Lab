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
#include "main.h" //allows us to specify data types such as uint32
#include "arm_math.h"
#include "math.h"
#include "utility.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "kalmanfilter_asm.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ITM_Port32(n) (*((volatile unsigned long *)(0xE0000000+4*n)))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
float TEST_ARRAY[] = {10.4915760032, 10.1349974709, 9.53992591829, 9.60311878706,
					10.4858891793, 10.1104642352, 9.51066931906, 9.75755656493,
					9.82154078273, 10.2906541933, 10.4861328671, 9.57321181356,
					9.70882714139, 10.4359069357, 9.70644021369, 10.2709894039,
					10.0823149505, 10.2954563443, 9.57130449017, 9.66832136479,
					10.4521677502, 10.4287240667, 10.1833650752, 10.0066049721,
					10.3279461634, 10.4767210803, 10.3790964606, 10.1937408814,
					10.0318963522, 10.4939180917, 10.2381858895, 9.59703103024,
					9.62757986516, 10.1816981174, 9.65703773168, 10.3905666599,
					10.0941977598, 9.93515274393, 9.71017053437, 10.0303874259,
					10.0173504397, 9.69022731474, 9.73902896102, 9.52524419732,
					10.3270730526, 9.54695650657, 10.3573960542, 9.88773266876,
					10.1685038683, 10.1683694089, 9.88406620159, 10.3290065898,
					10.2547227265, 10.4733422906, 10.0133952458, 10.4205693583,
					9.71335255372, 9.89061396699, 10.1652744131, 10.2580948608,
					10.3465431058, 9.98446410493, 9.79376005657, 10.202518901,
					9.83867150985, 9.89532986869, 10.2885062658, 9.97748768804,
					10.0403923759, 10.1538911808, 9.78303667556, 9.72420149909,
					9.59117495073, 10.1716116012, 10.2015818969, 9.90650056596,
					10.3251329834, 10.4550120431, 10.4925749165, 10.1548177178,
					9.60547133785, 10.4644672766, 10.2326496615, 10.2279703226,
					10.3535284606, 10.2437410625, 10.3851531317, 9.90784804928,
					9.98208344925, 9.52778805729, 9.69323876912, 9.92987312087,
					9.73938925207, 9.60543743477, 9.79600805462, 10.4950988486,
					10.2814361401, 9.7985283333, 9.6287888922, 10.4491538991,
					9.5799256668};

//float measurement[] = {0,1,2,3,4};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
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
	HAL_Init();

	uint32_t size = sizeof(TEST_ARRAY)/sizeof(float);
	uint32_t isValid = 0;
	float asm_result = 0;

	struct self asm_kalman_state = {
				0.1, //q
				0.1, //r
				5, //x
				0.1,//p
				0 //k
	};
	struct self c_kalman_state = {
				0.1, //q
				0.1, //r
				5, //x
				0.1,//p
				0 //k
	};
	struct self cmsis_kalman_state = {
				0.1, //q
				0.1, //r
				5.0, //x
				0.1,//p
				0 //k
	};
//  /* USER CODE END 1 */

	struct statistics c_stats = {0.0, 0.0, 0.0}; //last element left blank
	struct statistics cmsis_stats = {0.0, 0.0, 0.0}; //last element left blank

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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		ITM_Port32(31) = 1; //to be used to measure performance in speed
	    //ASM Implementation
		for(uint32_t i=0; i<size; i++){
			  kalmanfilter(&asm_kalman_state, &TEST_ARRAY[i], &isValid);
			  if(isValid){
				  asm_result = asm_kalman_state.x;
				  printf("Result (self.x) = %f\n",asm_result);
			  }else{
				  printf("Invalid = %ld\n",isValid);
			  }
		}
		ITM_Port32(31) = 2;
		//C implementation
		for(uint32_t i=0; i<size; i++){
			c_kalmanfilter(&c_kalman_state, &TEST_ARRAY[i]);
			c_stats.outputArray[i] = c_kalman_state.x;
			c_stats.inputArray[i] = TEST_ARRAY[i];
		}
		//CMSIS-DSP
		ITM_Port32(31) = 3;
//		cmsis_kalmanfilter(&cmsis_kalman_state, &TEST_ARRAY, &cmsis_stats, size);
		for(uint32_t i=0; i<size; i++){
					c_kalmanfilter(&cmsis_kalman_state, &TEST_ARRAY[i]);
					cmsis_stats.outputArray[i] = cmsis_kalman_state.x;
					cmsis_stats.inputArray[i] = TEST_ARRAY[i];
		}
		ITM_Port32(31) = 4;

		//C Ananlysis
		c_analysis(&c_stats,&TEST_ARRAY,&c_stats.outputArray);
		//CMSIS Analysis
		cmsis_analysis(&cmsis_stats, size);
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
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 40000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
