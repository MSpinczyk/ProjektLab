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
#include <stdlib.h>
#include "LIS35DE.h"
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
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static int TimerCnt = 0;
static int TimerCnt2 = 0;
volatile uint32_t click_cnt;
uint16_t  ADCRes;
int disp = 1;
int licznik = 0;
int x = 1;
int counter = 0;
int d1 = 10;
int d2 = 11;
int d3 = 12;
int d4 = 13;
uint16_t ADCDMABuff[3];
uint16_t VRef;
uint16_t Temp;
uint16_t ADCRes;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t SPITx[10];
uint16_t SPIRx[10];
uint8_t SPIRead( uint8_t Address ) {
	SPITx[0] = (0x80 | Address) << 8;
	HAL_GPIO_WritePin(GPIOA, SPI_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, SPITx, SPIRx, 1, 100);
	HAL_GPIO_WritePin(GPIOA, SPI_NSS_Pin, GPIO_PIN_SET);
	return( (uint8_t)(SPIRx[0] & 0xFF) );
}
void SPIWrite( uint8_t AddressAndAtributes, uint8_t Data  ) {
	SPITx[0] = (AddressAndAtributes << 8) + Data;
	HAL_GPIO_WritePin(GPIOA, SPI_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, SPITx, SPIRx, 1, 100);
	HAL_GPIO_WritePin(GPIOA, SPI_NSS_Pin, GPIO_PIN_SET);
}

static const uint16_t b7SegmentTable[16] =
{
		0X3F, /* 0 */
		0X06, /* 1 */
		0X5B, /* 2 */
		0X4F, /* 3 */
		0X66, /* 4 */
		0X6D, /* 5 */
		0X7D, /* 6 */
		0X07, /* 7 */
		0X7F, /* 8 */
		0X6F, /* 9 */
		0X77, /* A */
		0X7C, /* B */
		0X39, /* C */
		0X5E, /* D */
		0X79, /* E */
		0X71 /* F */
};
void LED(int ind){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);
	switch(ind){
		case 1:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
				break;
		case 2:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_6, GPIO_PIN_SET);
				break;
		case 3:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_SET);
			   	break;
		case 4:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
				break;
		case 5:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);
				break;
		}
}
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}
void display(int num, int ind){
	   HAL_GPIO_WritePin(GPIOC, S7Com4_Pin, GPIO_PIN_SET);
	   HAL_GPIO_WritePin(GPIOC, S7Com3_Pin, GPIO_PIN_SET);
	   HAL_GPIO_WritePin(GPIOC, S7Com2_Pin, GPIO_PIN_SET);
	   HAL_GPIO_WritePin(GPIOC, S7Com1_Pin, GPIO_PIN_SET);
	   switch(ind){
			case 1:
				   HAL_GPIO_WritePin(GPIOC, S7Com1_Pin, GPIO_PIN_RESET);
				   break;
			case 2:
				   HAL_GPIO_WritePin(GPIOC, S7Com2_Pin, GPIO_PIN_RESET);
				   break;
			case 3:
				   HAL_GPIO_WritePin(GPIOC, S7Com3_Pin, GPIO_PIN_RESET);
				   break;
			case 4:
				   HAL_GPIO_WritePin(GPIOC, S7Com4_Pin, GPIO_PIN_RESET);
				   break;
		}
	   HAL_GPIO_WritePin(GPIOC, S7A_Pin, b7SegmentTable[num] & 1);
	   HAL_GPIO_WritePin(GPIOC, S7B_Pin, b7SegmentTable[num] & 2);
	   HAL_GPIO_WritePin(GPIOC, S7C_Pin, b7SegmentTable[num] & 4);
	   HAL_GPIO_WritePin(GPIOC, S7D_Pin, b7SegmentTable[num] & 8);
	   HAL_GPIO_WritePin(GPIOC, S7E_Pin, b7SegmentTable[num] & 16);
	   HAL_GPIO_WritePin(GPIOC, S7F_Pin, b7SegmentTable[num] & 32);
	   HAL_GPIO_WritePin(GPIOC, S7G_Pin, b7SegmentTable[num] & 64);
	}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if ( (htim == &htim1) && (x%3==0) ){
		counter = 0;
		 switch (disp) {
				  case 1:
					  display(ADCRes % 10, 4);
					  disp++;
					  break;
				  case 2:
					  display((ADCRes % 100) / 10, 3);
					  disp++;
					  break;
				  case 3:
					  display((ADCRes % 1000)/100, 2);
					  disp++;
					  break;
				  case 4:
					  display(ADCRes/1000, 1);
					  disp = 1;
					  break;
				  }
		TimerCnt++;
		if (ADCRes > 2000){
			LED(2);
		}
		else{
			LED(1);
		}

		switch (TimerCnt) {
			case 1000:
				HAL_ADC_Start(&hadc1);
				HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
				ADCRes = HAL_ADC_GetValue(&hadc1);                    // 0-4095 == 0-3.3V
				ADCRes = (uint16_t)(3.3 * (double)ADCRes / 4.095);    // [mV]
				printf("ADC: %d \r\n", ADCRes);
				TimerCnt = 0;
				  }
  }
	else if ( (htim == &htim1) && (x%3==1) ){
		 switch (disp) {
				  case 1:
					  display(counter % 10, 4);
					  disp++;
					  break;
				  case 2:
					  display((counter % 100) / 10, 3);
					  disp++;
					  break;
				  case 3:
					  display((counter % 1000)/100, 2);
					  disp++;
					  break;
				  case 4:
					  display(counter/1000, 1);
					  disp = 1;
					  break;
				  }
		 LED(3);
		 TimerCnt2++;
		 switch (TimerCnt2) {
		 			case 1000:
//						HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCDMABuff,  3);
//						HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//						uint16_t VRef, Temp, ADCRes;
						ADCRes = ADCDMABuff[0];
						ADCRes = (uint16_t)(3.3 * (double)ADCRes / 4.095);

						Temp = ADCDMABuff[1];
						Temp = (float)(((3.3 * Temp/4.095 - 697)/2.5)*100);

						VRef = ADCDMABuff[2];
						VRef = (uint16_t)(3.3 * (double)VRef / 4.095);

						printf("%d %d %d \r\n", (uint16_t)ADCRes, (uint16_t)Temp, (uint16_t)VRef);
//						printf("%d %d %d \r\n", ADCDMABuff);
						counter++;
						TimerCnt2 = 0;
			  }

 }
	else if ( (htim == &htim1) && (x%3==2) ){
		counter = 0;
//		switch (disp) {
//			  case 1:
//				  display(d1, 1);
//				  disp++;
//				  break;
//			  case 2:
//				  display(d2, 2);
//				  disp++;
//				  break;
//			  case 3:
//				  display(d3, 3);
//				  disp++;
//				  break;
//			  case 4:
//				  display(d4, 4);
//				  disp = 1;
//				  break;
//			  }
		switch (disp) {
			  case 1:
				  display(click_cnt % 10, 4);
				  disp++;
				  break;
			  case 2:
				  display((click_cnt % 100) / 10, 3);
				  disp++;
				  break;
			  case 3:
				  display((click_cnt % 1000)/100, 2);
				  disp++;
				  break;
			  case 4:
				  display(click_cnt/1000, 1);
				  disp = 1;
				  break;
			  }
		uint8_t data_x, data_y, data_z;

//		data_x = SPIRead(LIS35DE_OUTX);
//		data_y = SPIRead(LIS35DE_OUTY);
//		data_z = SPIRead(LIS35DE_OUTZ);
//		printf("%d %d %d \r\n", (int)data_x, (int)data_y, (int)data_z);
		TimerCnt++;
		switch (TimerCnt) {
			case 500:
				LED(4);
				data_x = SPIRead(LIS35DE_OUTX);
				data_y = SPIRead(LIS35DE_OUTY);
				data_z = SPIRead(LIS35DE_OUTZ);
				printf("%d %d %d \r\n", (int)data_x, (int)data_y, (int)data_z);
//				d1++;
//				d2++;
//				d3++;
//				d4++;
//				if (d1 == 16){
//					d1 = 10;
//				}
//				if (d2 == 16){
//					d2 = 10;
//				}
//				if (d3 == 16){
//					d3 = 10;
//				}
//				if (d4 == 16){
//					d4 = 10;
//				}

				break;
			case 1000:
				LED(5);
				TimerCnt = 0;
				break;
		}
	}
	if (htim == &htim10){
		if ((HAL_GPIO_ReadPin(GPIOB, Button1_Pin) == 0) || (HAL_GPIO_ReadPin(GPIOB, Button2_Pin) == 0)){
					licznik++;
				}
		else licznik = 0;
	}

}
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	if( htim == &htim1 )
//	{
//		uint8_t data_x, data_y, data_z;
//
//		data_x = SPIRead(LIS35DE_OUTX);
//		data_y = SPIRead(LIS35DE_OUTY);
//		data_z = SPIRead(LIS35DE_OUTZ);
//
//		printf("%d %d %d \r\n", (int)data_x, (int)data_y, (int)data_z);
//
//		//time = display_show_number(ADCRes, DISPLAY_FREQ_250, time);
//	}
//}
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
  MX_USART2_UART_Init();
  MX_TIM10_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_ADC_Start_IT(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCDMABuff,  3);
  HAL_ADC_Start(&hadc1);
  SPIWrite(LIS35DE_CTRL_REG1, LIS35DE_CTRL_REG1_DR | LIS35DE_CTRL_REG1_PD | LIS35DE_CTRL_REG1_ZEN |
		  	  	  	  	  	  LIS35DE_CTRL_REG1_YEN | LIS35DE_CTRL_REG1_XEN);
  SPIWrite(LIS35DE_CTRL_REG2, LIS35DE_CTRL_REG2_BOOT);
  SPIWrite(LIS35DE_CTRL_REG3, LIS35DE_CTRL_REG3_IHL | LIS35DE_CTRL_REG3_I1CFG0 |
		  	  	  	  	  	  LIS35DE_CTRL_REG3_I1CFG1 | LIS35DE_CTRL_REG3_I1CFG2);

  SPIWrite(LIS35DE_CLICK_CFG, LIS35DE_CLICK_CFG_DOUBLE_X | LIS35DE_CLICK_CFG_DOUBLE_Y | LIS35DE_CLICK_CFG_DOUBLE_Z);
  SPIWrite(LIS35DE_CLICK_THSY_X, 0xF1);
  SPIWrite(LIS35DE_CLICK_THSZ, 0x01);

  SPIWrite(LIS35DE_CLICK_TIMELIMIT, 0xFF);
  SPIWrite(LIS35DE_CLICK_LATENCY, 0x7F);
  SPIWrite(LIS35DE_CLICK_WINDOW, 0xFF);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if ((licznik>5) && (licznik<255)){
		  if(HAL_GPIO_ReadPin(GPIOB, Button1_Pin) == 0){
			  if (x == 0){
				  x = 0;
				  licznik = 255;
			  }
			  else if (x > 0){
				  x--;
				  licznik = 255;
			  }
		  }
		  else if ((HAL_GPIO_ReadPin(GPIOB, Button2_Pin) == 0)){
			  if (x == 2){
				  x = 2;
				  licznik = 255;
			  }
			  else if (x <2){
				  x++;
				  licznik = 255;
			  }
		  }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the analog watchdog
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.HighThreshold = 2000;
  AnalogWDGConfig.LowThreshold = 0;
  AnalogWDGConfig.Channel = ADC_CHANNEL_1;
  AnalogWDGConfig.ITMode = ENABLE;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 100;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 99;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, S7G_Pin|S7D_Pin|S7E_Pin|S7C_Pin
                          |S7B_Pin|S7F_Pin|S7A_Pin|S7DP_Pin
                          |S7Com4_Pin|S7Com3_Pin|S7Com2_Pin|S7Com1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S7G_Pin S7D_Pin S7E_Pin S7C_Pin
                           S7B_Pin S7F_Pin S7A_Pin S7DP_Pin
                           S7Com4_Pin S7Com3_Pin S7Com2_Pin S7Com1_Pin */
  GPIO_InitStruct.Pin = S7G_Pin|S7D_Pin|S7E_Pin|S7C_Pin
                          |S7B_Pin|S7F_Pin|S7A_Pin|S7DP_Pin
                          |S7Com4_Pin|S7Com3_Pin|S7Com2_Pin|S7Com1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_NSS_Pin */
  GPIO_InitStruct.Pin = SPI_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Button1_Pin Button2_Pin */
  GPIO_InitStruct.Pin = Button1_Pin|Button2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ClickInterrupt_Pin */
  GPIO_InitStruct.Pin = ClickInterrupt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ClickInterrupt_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_15)
  {
	  click_cnt += 1;
  }
}
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
