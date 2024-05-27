/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
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

void SEND_MESSAGE(){ //Mandar una secuencia de datos u otra segun el mapa activo
    TxHeader.DLC = 1; //Data length (envio 2 bytes)
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.StdId = 0x558; //ID (identificador del enviador)

    switch (mapaACT){
    case 1:
    	TxData[0] = 0x01;
    	break;
    case 2:
    	TxData[0] = 0x02;
    	break;
    case 3:
    	TxData[0] = 0x04;
    	break;
    case 4:
    	TxData[0] = 0x08;
    	break;
    case 5:
    	TxData[0] = 0x10;
    	break;
    }

	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}//<-- falta traction y launch control

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){ //Cambio de mapa activo segun el botón que se pulse
	if (GPIO_Pin == GPIO_PIN_1){ //Al pulsar el boton gris se sube de mapa
		if (mapaACT < 5){
			mapaACT++;
		} else {
			mapaACT = 1;
		}
	}

	if (GPIO_Pin == GPIO_PIN_2){ //Al pulsar el boton amarillo se baja de mapa
		if (mapaACT > 1 ){
			mapaACT--;
		} else {
			mapaACT = 5;
		}
	}
	SEND_MESSAGE();
} //<-- falta debouncer y traction y launch control

void RECEIVE_MESSAGE(){
	switch (RxData[0]){
	    case 0x81:
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, 0);
	    	break;
	    case 0x82:
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, 0);
	    	break;
	    case 0x84:
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, 0);
	    	break;
	    case 0x88:
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, 0);
	    	break;
	    case 0x90:
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
	    	break;
	}

	switch (RxData[1]){
	    case 0x80:
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 0);
	    	break;
	    case 0x81:
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 0);
	    	break;
	    case 0x82:
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 0);
	    	break;
	    case 0x83:
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 1);
	    	break;
	}


}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){ //Para recibir mensajes del bus //<-- REVISAR hcan o hcan1
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	if (RxHeader.DLC == 2){
		RECEIVE_MESSAGE();

	}
}

// Temporizador del heartbeat
/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM6){
		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12); // Va ha hacer toggle cada segundo (porque: 1/(timer/(prescaler*period)) = 1/(84M/(42k*2k)) = 1s)
		TxHeader.DLC = 2; //Data length (envio 2 bytes)
		TxHeader.StdId = 0x558; //ID (identificador del enviador)
		TxData[0] = 200; //ms delay
		TxData[1] = 4; //loop rep

		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox); //Enviar el mensaje
	}
}
*/
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
  /* USER CODE BEGIN 2 */
  	 HAL_CAN_Start(&hcan1);

     //Activar la notificacion
     HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);

//     HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //Estas 4 lineas para comprobar que funciona el bus - mensajes continuos
//	  TxData[0] = 100; //ms delay
//	  TxData[1] = 8; //loop rep
//	  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
//	  HAL_Delay(1000);



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
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef canfilterconfig;

        canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
        canfilterconfig.FilterBank = 18; //which filter bank to use from the assigned ones
        canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
        canfilterconfig.FilterIdHigh = 0x103<<5;
        canfilterconfig.FilterIdLow = 0x0000;
        canfilterconfig.FilterMaskIdHigh = 0x103<<5; //Escribo en la posicion 5
        canfilterconfig.FilterMaskIdLow = 0x0000;
        canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
        canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
        canfilterconfig.SlaveStartFilterBank = 20; //how many filters to assign to the CAN1 (master can)

        HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* USER CODE END CAN1_Init 2 */

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

  /*Configure GPIO pins : PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
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
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

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
