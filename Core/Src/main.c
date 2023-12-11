/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
uint8_t Rxbuff[57], buff[8];
uint8_t Txbuff[11];
uint16_t ADC_Val;
float Idc;
float Kps, Kis, Kds, Kpi, Kii, Kdi, Iref = 1.5;
float setpoint;
uint8_t enable = 0, ramp_enable = 0;
uint8_t temp[11];
float speed, pwm;
#define T 0.01
#define Idm 1.5
#define Round_to_pulse 1900 // 1900 xung/v
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
//-------------------------- UART2 interupt------------------------------------/
// DONE
float ASCII2float(uint8_t *data_recv, int n)
{
	// Example buff = {'4','5','.','7','8','0','0','0'}
	int nega = 0, i = 0, comma = 0, j = 0;
	float inter = 0, frac = 0;
	for(i=0;i<n;i++)
	{
		buff[i] = data_recv[i];
	}
	//if (buff[0] == '-') nega = 1;
	i=0;
	// Read abs of buff
	do
	{
		if (buff[i+nega] == '-')
		{
			nega = 1;
		}
		if(buff[i+nega] == '.')
		{
			comma = 1;
			break;
		}
		// inter = 45, i = 0,1,2
		inter = inter*10 + (buff[i+nega] - 0x30);
		i++;
	} while(i+nega<n);
	if(comma)
	{
		// j = 7,6,5,4,3
		for(j = n-1;j>i+nega;j--)
		{
			frac = frac + buff[j] - 0x30;
			frac = frac/10;
		}
	}
	if(nega) return -(inter+frac);
	else return (inter+frac);
}
//DONE
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
	if(huart->Instance == USART2)
	{
		//HAL_UART_Transmit(&huart2, Txbuff, 20, 200);
		// Run callback from GUI
		// frame truyen: Command(1 byte) - setpoint(8 bytes) - Kps(8 bytes) - Kis(8 bytes) - Kds(8 bytes) - Kpi(8 bytes) - Kii(8 bytes) - Kdi(8 bytes)
		int n = 8;
		if(Rxbuff[0] == 'R') enable = 1;
		if(Rxbuff[0] == 'S') enable = 0;
		if(Rxbuff[0] == 'V')
		{
			enable = 1;
			ramp_enable = 1;
		}
		if(Rxbuff[0] == 'P')
		{
			setpoint = ASCII2float(&Rxbuff[1], n)*Round_to_pulse/60;
			Kps = ASCII2float(&Rxbuff[n + 1], n);
			Kis = ASCII2float(&Rxbuff[2*n + 1], n);
			Kds = ASCII2float(&Rxbuff[3*n + 1], n);
			Kpi = ASCII2float(&Rxbuff[4*n + 1], n);
			Kii = ASCII2float(&Rxbuff[5*n + 1], n);
			Kdi = ASCII2float(&Rxbuff[6*n + 1], n);
		}
	  HAL_UART_Receive_IT(&huart2, Rxbuff, 7*n+1);
	}
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
}
//---------------------ADC1 interupt-----------------------------------------/
// DONE
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
	if(hadc->Instance == ADC1)
	{
		ADC_Val = HAL_ADC_GetValue(&hadc1);
		Idc = (float) (ADC_Val*3.3/4095 - 2.51)*7;
	}
  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADC_ConvCpltCallback must be implemented in the user file.
   */
}
// DONE
void send_data(float data)
{
	uint8_t i = 0;
	// Txbuff[1] is sign of data
	if(data < 0)
	{
		Txbuff[1] = '-';
		data = -data;
	}
	else Txbuff[1] = '+';
	// Send abs of data. Example data = 123.45
	// decimal = 123
	int decimal = (int) data;
	// frac = 45
	float frac = data - (int) data;
	int inter = 2;
	do 
	{
		// Temp = {'3','2','1'}, inter = 2,3,4,5 
		temp[inter] = decimal%10 + 0x30;
		inter++;
		decimal = decimal/10;
	} while(decimal!= 0 && inter<=8);
	// Inter = 5
	// Txbuff = {'type','sign','1','2','3'}
	for(i = 2; i < inter; i++)
	{
		Txbuff[i] = temp[inter - i + 1];
	}
	// Txbuff = {'type','sign','1','2','3','.'}, inter = 6
	Txbuff[inter] = '.';
	inter++;
	// inter = 6,7,8, Txbuff = {'type','sign','1','2','3','.','4','5','0','\r','\n'}
	while(inter <= 8)
	{
		Txbuff[inter] = ((int) (10*frac))%10 + 0x30;
		frac = frac * 10;
		inter++;
	}
	Txbuff[9] = '\r';
	Txbuff[10] = '\n';
	HAL_UART_Transmit(&huart2, Txbuff, 11, 1);
}
// DONE
void send_current(void)
{
	Txbuff[0] = 'I';
	send_data(Idc);
}
void send_set_speed(float set_speed)
{
	Txbuff[0] = 'V';
	send_data(set_speed);
}
// DONE
void send_pwm(void)
{
	Txbuff[0] = 'u';
	send_data(pwm * 100);
}
// DONE
void send_speed(void)
{
	Txbuff[0] = 'S';
	send_data((speed*60)/Round_to_pulse);
}
// DONE
void send_Iref(void)
{
	Txbuff[0] = 'R';
	send_data(Iref);
}
// DONE
float low_filter_speed(float respone)
{
	static float pre_respone = 0, pre_out = 0;
	float out;
	out = 0.7*pre_out + 0.3*pre_respone;
	pre_respone = respone;
	pre_out = out;
	return out;
}
// DONE
float low_filter_current(float respone)
{
	static float pre_respone = 0, pre_out = 0;
	float out;
	out = 0.8*pre_out + 0.2*pre_respone;
	pre_respone = respone;
	pre_out = out;
	return out;
}
// DONE
// Note: Noise of current, so don't use inner loop
void PID_current(void)
{
//	float ek, uk;
//	static float uk_1 = 0, ek_1 = 0, ek_2 = 0;
//	Idc = low_filter_current(Idc);
//	ek = Iref - Idc;
//	uk = uk_1 + Kpi*(ek - ek_1) + Kii*T*0.5*(ek + ek_1) + Kdi*(ek - 2*ek_1 + ek_2)/T;
//	uk_1 = uk;
//	ek_2 = ek_1;
//	ek_1 = ek;
//	// output duration
//	if(uk < -1) 
//	{
//		uk = -1;
//	}
//	if(uk > 1) 
//	{
//		uk = 1;
//	}
//	// Convert uk to pwm
//	pwm = 0.5*uk + 0.5 +0.01;
}
// DONE
// One loop control speed
void PID_speed(void)
{
	float ek;
	static float uk_1 = 0, ek_1 = 0, ek_2 = 0;
	// read speed from encoder timer 3
	speed = ((int16_t)(TIM3->CNT))/T; // pulse/s
	// pass it to low filter
	speed = low_filter_speed(speed);
	// clear counter of timer 3
	TIM3->CNT = 0;
  ek = setpoint - speed;
	pwm = uk_1 + Kps*(ek - ek_1) + Kis*T*0.5*(ek + ek_1) + Kds*(ek - 2*ek_1 + ek_2)/T;
	uk_1 = pwm;
	ek_2 = ek_1;
	ek_1 = ek;
	// pwm saturation
	if (pwm > 1) pwm = 1;
	else if(pwm < -1) pwm = -1;
	pwm = 0.48*pwm + 0.5;
}
void send_GUI(void)
{
	send_speed();
	//send_Iref();
	//send_current();
	send_pwm();
}
//------------------------- T sample interupt-------------------------------/
// DONE
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
	static int count = 0;
	static float timer_ramp = 0;
	float set_speed = 0;
	count++;
	if (count == 100)
	{
		count = 0;
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
	if(enable)
	{
		if(ramp_enable == 1)
		{
			timer_ramp = timer_ramp + 0.01;
			if(timer_ramp <= 1)
			{
				set_speed = 100*timer_ramp;
				setpoint = (set_speed * 1900)/60;
				send_set_speed(set_speed);
			}
			else if(timer_ramp > 2 && timer_ramp <= 3)
			{
				set_speed = 100+80*(timer_ramp-2);
				setpoint = (set_speed * 1900)/60;
				send_set_speed(set_speed);
			}
			else if(timer_ramp > 4 && timer_ramp <= 5)
			{
				set_speed = 180-90*(timer_ramp-4);
				setpoint = (set_speed * 1900)/60;
				send_set_speed(set_speed);
			}
			else if(timer_ramp > 6 && timer_ramp <= 7)
			{
				set_speed = 90-90*(timer_ramp-6);
				setpoint = (set_speed * 1900)/60;
				send_set_speed(set_speed);
			}
			else if(timer_ramp > 8 && timer_ramp <= 9)
			{
				set_speed = -90*(timer_ramp-8);
				setpoint = (set_speed * 1900)/60;
				send_set_speed(set_speed);
			}
			else if(timer_ramp > 10 && timer_ramp <= 11)
			{
				set_speed = -90-90*(timer_ramp-10);
				setpoint = (set_speed * 1900)/60;
				send_set_speed(set_speed);
			}
			else if(timer_ramp > 12 && timer_ramp <= 13)
			{
				set_speed = -180+80*(timer_ramp-12);
				setpoint = (set_speed * 1900)/60;
				send_set_speed(set_speed);
			}
						else if(timer_ramp > 14 && timer_ramp <= 15)
			{
				set_speed = -100+100*(timer_ramp-14);
				setpoint = (set_speed * 1900)/60;
				send_set_speed(set_speed);
			}
			else if(timer_ramp >= 16)
			{
				enable = 0;
				ramp_enable = 0;
				timer_ramp = 0;
			}
		}
	//HAL_ADC_Start_IT(&hadc1);
	PID_speed();
	//PID_current();
	TIM1->CCR2 = (int)(TIM1->ARR * pwm);
	if(count % 5 == 0)
		{			
			send_GUI();
		}
	}
	else 
	{
		TIM1->CCR2 = (int)(TIM1->ARR * 0.5);
		
//		//----- DEBUG----///
//		HAL_ADC_Start_IT(&hadc1);
//		TIM1->CCR2 = (int)(TIM1->ARR * 1);
//		// read speed from encoder timer 3
		//speed = ((int16_t)(TIM3->CNT))/T; // pulse/s
		//TIM3->CNT = 0;
//		// pass it to low filter
//		speed = low_filter_speed(speed);
//		Idc = low_filter_current(Idc);
		//send_GUI();
	}
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
}
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	// Enable PWM
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	// Enable T sample
	HAL_TIM_Base_Start_IT(&htim2);
	// Enable encoder mode
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	// Enable Rx_UART 2 interupt
	HAL_UART_Receive_IT(&huart2, Rxbuff, 7*8+1);
  /* USER CODE END 2 */
	//---------------- DEBUG---------------//
//	TIM1->CCR2 = 999;
//	HAL_Delay(3000);
//	HAL_ADC_Start_IT(&hadc1);
//	speed = (signed)(TIM3->CNT);
//	TIM1->CCR2 = 502;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 719;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
