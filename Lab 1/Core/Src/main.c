/* Includes ------------------------------------------------------------------*/
#define ARM_MATH_CM4
#include "main.h" //allows us to specify data types such as uint32
#include "arm_math.h"

/* Private includes ----------------------------------------------------------*/
#include "kalmanfilter_asm.h"
#include "stdio.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

typedef struct self {
			  float q;
			  float r;
			  float x;
			  float p;
			  float k;
}self;

float TEST_ARRAY[] = {10.4915760032, 10.1349974709, 9.53992591829, 9.60311878706,
//					10.4858891793, 10.1104642352, 9.51066931906, 9.75755656493,
//					9.82154078273, 10.2906541933, 10.4861328671, 9.57321181356,
//					9.70882714139, 10.4359069357, 9.70644021369, 10.2709894039,
//					10.0823149505, 10.2954563443, 9.57130449017, 9.66832136479,
//					10.4521677502, 10.4287240667, 10.1833650752, 10.0066049721,
//					10.3279461634, 10.4767210803, 10.3790964606, 10.1937408814,
//					10.0318963522, 10.4939180917, 10.2381858895, 9.59703103024,
//					9.62757986516, 10.1816981174, 9.65703773168, 10.3905666599,
//					10.0941977598, 9.93515274393, 9.71017053437, 10.0303874259,
//					10.0173504397, 9.69022731474, 9.73902896102, 9.52524419732,
//					10.3270730526, 9.54695650657, 10.3573960542, 9.88773266876,
//					10.1685038683, 10.1683694089, 9.88406620159, 10.3290065898,
//					10.2547227265, 10.4733422906, 10.0133952458, 10.4205693583,
//					9.71335255372, 9.89061396699, 10.1652744131, 10.2580948608,
//					10.3465431058, 9.98446410493, 9.79376005657, 10.202518901,
//					9.83867150985, 9.89532986869, 10.2885062658, 9.97748768804,
//					10.0403923759, 10.1538911808, 9.78303667556, 9.72420149909,
//					9.59117495073, 10.1716116012, 10.2015818969, 9.90650056596,
//					10.3251329834, 10.4550120431, 10.4925749165, 10.1548177178,
//					9.60547133785, 10.4644672766, 10.2326496615, 10.2279703226,
//					10.3535284606, 10.2437410625, 10.3851531317, 9.90784804928,
//					9.98208344925, 9.52778805729, 9.69323876912, 9.92987312087,
//					9.73938925207, 9.60543743477, 9.79600805462, 10.4950988486,
//					10.2814361401, 9.7985283333, 9.6287888922, 10.4491538991,
					9.5799256668};



void cmsis_kalmanfilter(self *state, float *InputArray, float *DSP_OutputArray, uint32_t *isValid, uint32_t size){
	//temporary variables
	float x = 0; //self.p + self.r
	float y = 0; //measurement - self.x
	float z = 0; //self.k *(measurement -self.x)
	float m = 0; //(1-self.k)
	float constant = 1;
	for(uint32_t i = 0; i<size; i++){
		//perform kalman update
		arm_add_f32(&state->p,&state->q, &state->p, 1); //self.p = self.p + self.q
		arm_add_f32(&state->p,&state->r, &x, 1); //x = self.p + self.r
		(state->k) = (state->p)/x; //self.k = self.p / x
		arm_sub_f32(&InputArray[i],&state->x, &y, 1); //y = measurement - self.x
		arm_mult_f32(&state->k,&y, &z, 1); //z = self.k * y
		arm_add_f32(&state->x,&z, &state->x, 1);//self.x = self.x + z
		arm_sub_f32(&constant,&state->k, &m, 1);//m = (1-self.k)
		arm_mult_f32(&m,&state->q, &state->p, 1); //self.p = m*self.p
		//store output values
		*(DSP_OutputArray + i) = state->x;
	}
}


int main(void){
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	uint32_t size = sizeof(TEST_ARRAY)/sizeof(float);
	struct self teststruct = {
				0.1, //q
				0.1, //r
				5, //x
				0.1,//p
				0 //k
	};
	uint32_t isValid = 0;
//	float asm_result = 0;
//	float cmsis_result = 0;
	float DSP_OutputArray[size];
	float DSP_Diff[size];
	float DSP_STD;
	float DSP_Correlation;
	float DSP_Convolution;
	float DSP_AverageOfdiff;

  /* Configure the system clock */
	SystemClock_Config();

	while(1){
//		ITM_Port32(31)=1; //to be used to measure performance in speed
//		for(uint32_t i=0; i<size; i++){
//			  kalmanfilter(&teststruct, &TEST_ARRAY[i], &isValid);
//			  if(isValid){
//				  asm_result = teststruct.x;
//				  printf("Result (self.x) = %f\n",asm_result);
//			  }else{
//				  printf("Invalid = %ld\n",isValid);
//			  }
//		}
//		asm_result = teststruct.x;

//		ITM_Port32(31) = 2;
		cmsis_kalmanfilter(&teststruct, &TEST_ARRAY, DSP_OutputArray, &isValid, size);

		arm_sub_f32(&TEST_ARRAY,&DSP_OutputArray,&DSP_Diff,size); //calculate (Input stream - Output stream)
		arm_std_f32(&DSP_Diff,size,&DSP_STD); //calculate standard deviation of diff
		arm_mean_f32(&DSP_Diff,size,&DSP_AverageOfdiff); //calculate average of diff
		arm_conv_f32(&TEST_ARRAY, size, &DSP_OutputArray, size, &DSP_Convolution); //calculate convolution of Input & Output stream
		arm_correlate_f32(&TEST_ARRAY, size, &DSP_OutputArray, size, &DSP_Correlation); //calculate correlation of Input & Output stream
	}
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

