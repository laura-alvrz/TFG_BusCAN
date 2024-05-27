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
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8];
uint8_t RxData[8];

uint32_t TxMailbox;

uint8_t datacheck = 0;
uint8_t mapaACT = 1;

uint8_t TC = 0; //Traction control
uint8_t LC = 0; //Launch control

void SEND_MESSAGE(){ //Mandar una secuencia de datos u otra segun el mapa activo
	 TxHeader.DLC = 2; //Data length
	 TxHeader.IDE = CAN_ID_STD;
	 TxHeader.RTR = CAN_RTR_DATA;
	 TxHeader.StdId = 0x150; //ID

    switch (mapaACT){
    case 1:
    	TxData[0] = 0x81;
    	break;
    case 2:
    	TxData[0] = 0x82;
    	break;
    case 3:
    	TxData[0] = 0x84;
    	break;
    case 4:
    	TxData[0] = 0x88;
    	break;
    case 5:
    	TxData[0] = 0x90;
    	break;
    }


    if (TC == 0 && LC == 0){
    	TxData[1] = 0x80;
    } else if (TC == 1 && LC == 0){
    	TxData[1] = 0x81;
    } else if (TC == 0 && LC == 1){
        TxData[1] = 0x82;
    } else if (TC == 1 && LC == 1){
    	TxData[1] = 0x83;
    }

	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

void RECEIVE_MESSAGE(){
	switch (RxData[0]){
	    case 0x01:
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, 0);
	    	mapaACT = 1;
	    	break;
	    case 0x02:
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
	    	mapaACT = 2;
	    	break;
	    case 0x04:
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
	    	mapaACT = 3;
	    	break;
	    case 0x08:
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, 0);
	    	mapaACT = 4;
	    	break;
	    case 0x10:
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, 1);
	    	mapaACT = 5;
	    	break;
	}

	switch (RxData[1]){
		case 0x00:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 0);
			TC = 0;
			LC = 0;
			break;
		case 0x01: //TC enabled
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 0);
			TC = 1;
			LC = 0;
			break;
		case 0x02: //LC enabled
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 1);
			TC = 0;
			LC = 1;
			break;
		case 0x03: //TC y LC enabled
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 1);
			TC = 1;
			LC = 1;
			break;
	}
	SEND_MESSAGE();
}

// Desactivar LC al pulsar el PA0
// No activar el LC si está activo el mapa 5


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){// Heartbeat
	if(htim->Instance==TIM2){
	    TxHeader.DLC = 0; //Data length
	    TxHeader.IDE = CAN_ID_STD;
	    TxHeader.RTR = CAN_RTR_DATA;
	    TxHeader.StdId = 0x100; //ID (identificador del enviador)

	    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_8); //Heartbeat

	    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
	    HAL_TIM_Base_Stop_IT(&htim2);

	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){ //Para recibir mensajes del bus //<-- REVISAR hcan o hcan1
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData);
	if (RxHeader.DLC == 2){
		RECEIVE_MESSAGE();
	}
	if (RxHeader.StdId == 0x600){ //Heartbeat
		HAL_TIM_Base_Start_IT(&htim2);
	}
}




/*void check_CAN_error_status(void) {
    uint32_t error = HAL_CAN_GetError(&hcan1);
    if (error != HAL_CAN_ERROR_NONE) {
        for(int i = 0; i <10; i++){
        	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
        }
    }
}*/
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
  MX_CAN1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);

    //Activar la notificacion
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);

    //Inicialmente esta activado el mapa 1
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	 // check_CAN_error_status();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 21;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef canfilterconfig;

    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank = 10; //which filter bank to use from the assigned ones
    canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    canfilterconfig.FilterIdHigh = 0x600<<5; //0110 0000 0000 // Lo que se compara con los ID entrantes
    canfilterconfig.FilterIdLow = 0x0000;
    canfilterconfig.FilterMaskIdHigh = 0x600<<5; //Escribo en la posicion 5 // Solo se comparan los bit del FilterID con los ID entrantes que en el FilterMask sean 1
    canfilterconfig.FilterMaskIdLow = 0x0000;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilterconfig.SlaveStartFilterBank = 18;

    HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* USER CODE END CAN1_Init 2 */

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
  htim2.Init.Prescaler = 42000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
