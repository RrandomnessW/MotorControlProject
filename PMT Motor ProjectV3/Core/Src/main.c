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

  Index code dictionary:
  0x01: Control Parameters
  0x10: Current Values
  0x11: Velocity Values
  0x12: Position Values
  0x36: Velocity Loop Control Parameters (PID)
  0xD0: Control Loop Configuration Parameters
  0xFE: calculating CRC of data or header or smth.

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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

union ParaType
{
	float flt;
	int8_t byte[4];
} Kp, Ki, Kd, Ka;

union MeasuredType
{
	int32_t INT;
	uint8_t byte[4];
} Vel_Measured, Vel_Target;

#define DMA_RX_BUFFER_SIZE 256
uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];

#define DMA_TX_BUFFER_SIZE 520
uint8_t DMA_TX_Buffer[DMA_TX_BUFFER_SIZE];

uint8_t Rx_Head[8], Rx_Data[20], Rx_Buffer[256], Data_Buffer[256];
uint8_t Rply_Head[8] = { 0xA5, 0xFF, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00 };
uint8_t Rply_Data[512];
uint8_t Tx_Buffer[512];
volatile uint16_t counter = 0, Rx_Counter;
volatile uint8_t COM_Status; // status = 0, DMA not finished. = 1, DMA finished
volatile uint8_t uartTxDone;
uint8_t message[29] = "Driver Ware has began running";
uint16_t EncVal, direction;
int32_t Speed_TIM6_Measured, Speed_TIM6_Target, Err, Err_1, Err_2, Uk, Uk_1, dU;
uint16_t i, j, CRC_Val, x, y, accumulator, k;
float vKp, vKi, vKd, vKa;
uint8_t CCW = 0;

static const uint16_t crctable[256] = {
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
	0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
	0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
	0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
	0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
	0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
	0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
	0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
	0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
	0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
	0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
	0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
	0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
	0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
	0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
	0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
	0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
	0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
	0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
	0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
	0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
	0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
	0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM15_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Transmit_DMA(UART_HandleTypeDef *UartHandle, uint8_t* aTxBuffer, uint16_t TXBUFFERSIZE);
uint16_t CalculateCRC16( const uint8_t* c_ptr, size_t len );
void Make_Rply_Head(uint8_t* c_ptr, uint8_t r2, uint8_t r3, uint8_t r4, uint8_t r5);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM15_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit (&huart1, message, strlen(message), 100 );

  __HAL_UART_ENABLE_IT( &huart1, UART_IT_IDLE );//enable idle interrupt
  __HAL_DMA_ENABLE_IT( &hdma_usart1_rx, DMA_IT_TC);//enable DMA complete interrupt
  __HAL_DMA_DISABLE_IT( &hdma_usart1_rx, DMA_IT_HT);//disable half transfer interrupt

  HAL_UART_Receive_DMA( &huart1, DMA_RX_Buffer, DMA_RX_BUFFER_SIZE );//enable uart1 DMA receiving

  COM_Status = 0;

  HAL_TIM_PWM_Start( &htim15, TIM_CHANNEL_1 );
  HAL_TIMEx_PWMN_Start( &htim15, TIM_CHANNEL_1 );

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  TIM2->CNT = 32767; //encoder couter offset;
  HAL_TIM_Base_Start(&htim2);

  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



	  //Transmit_DMA( &huart1, "HELLO\r\n", 7);

	  if( COM_Status == 1 )
	  {

		  Transmit_DMA( &huart1, Data_Buffer, strlen(Data_Buffer) );
		  Transmit_DMA( &huart1, "\n\r", 2 );

		  for( i = 0; i < Rx_Counter; ++i )
		  {

			  if ( Data_Buffer[i] == 0xA5 )
			  {
				  //Transmit_DMA( &huart1, "this works\r\n", 12);
				  //Transmit_DMA( &huart1, "\n\r", 2 )
				  for ( j = 0; j < 8; ++j )
				  {
					  Rx_Head[j] = Data_Buffer[i + j];
				  }
				  //do a crc check
				  CRC_Val = CalculateCRC16( Rx_Head, 6 );
				  uint8_t write [2] = { (CRC_Val >> 8) & 0x00FF, CRC_Val & 0x00FF };
				  Transmit_DMA( &huart1, write, sizeof(write) );

				  if ( ( (CRC_Val >> 8) & 0x00FF ) != Rx_Head[6] || ( CRC_Val & 0x00FF ) != Rx_Head[7] )
				  {
					  //prepare error reply head and exit the loop checking the data buffer
					  Make_Rply_Head( Rply_Head, (Rx_Head[2] & 0xFC), 0x08, 0x0, 0x0);
					  Transmit_DMA( &huart1, Rply_Head, sizeof(Rply_Head) );
					  COM_Status = 0;
					  break;
				  }

				  //if Rx_Head is correct, check if data follows using the "length" byte (Rx_Head[5])
				  if ( Rx_Head[5] != 0 )
				  {
					  for ( j = 0; j < 2*Rx_Head[5] + 2 ; ++j )
					  {
						  Rx_Data[j] = Data_Buffer[ i + 8 + j ];
					  }
					  //check CRC
					  CRC_Val = CalculateCRC16( Rx_Data, Rx_Head[5]*2 ); //"length" is measured in words. Each word = 2 bytes

					  uint8_t write [2] = { (CRC_Val >> 8) & 0x00FF, CRC_Val & 0x00FF };
					  Transmit_DMA( &huart1, write, sizeof(write) );

					  if ( ( (CRC_Val >> 8) & 0x00FF ) != Rx_Data[ Rx_Head[5]*2] || ( CRC_Val & 0x00FF ) != Rx_Data[ Rx_Head[5]*2 + 1 ] )
					  {
						  Make_Rply_Head( Rply_Head, (Rx_Head[2] & 0xFC), 0x08, 0x0, 0x0);
						  Transmit_DMA( &huart1, Rply_Head, sizeof(Rply_Head) );
						  COM_Status = 0;
						  break;
					  }
					  //Command is complete, Rply_Head[3] = 0x01 //1-command complete, 2-command incomplete, 4-invalid command, 6-do not have write access, 8-crc/frame error


				  }
				  //If data crc is correct/there is no data, continue on to the index.
				  //check Index
				  switch ( Rx_Head[3] )
				  {
				  case 0x01:
					  HAL_TIM_Base_Stop_IT(&htim6);
					  TIM1->CCR1 = 1607;
					  COM_Status = 0;
					  break;
				  case 0x10:
					  COM_Status = 0;
					  break;
				  case 0x11:
					  //for a more in-depth protocol: if offset is 04 -> set target velocity (in rpm)
					  //output current rpm
					  Make_Rply_Head( Rply_Head, ( (Rx_Head[2] & 0xFC) | 0x02 ), 0x01, 0, 2);

					  Vel_Measured.INT = (int32_t)( Speed_TIM6_Measured  ); //outputs rpm * ( 200.0 * 60.0 * (1.0/5120.0) )

					  for( k = 0; k < 4; ++k )
						  Rply_Data[k] = Vel_Measured.byte[3-k];
					  //most significant bit at the highest k/ at the end of the array.
					  CRC_Val = CalculateCRC16( Rply_Data, Rx_Head[5]*2 );
					  Rply_Data[5] = ( CRC_Val >> 8 ) & 0x00FF;
					  Rply_Data[6] = CRC_Val & 0x00FF;

					  for ( k = 0; k < 8; ++k )
						  Tx_Buffer[k] = Rply_Head[k];
					  for ( k = 0; k < 6; ++k )
						  Tx_Buffer[ k + 8 ] = Rply_Data[k];

					  Transmit_DMA( &huart1, Tx_Buffer, 14 );

					  switch( Rx_Head[4] )
					  {
					  case 0x00:
						  break;
					  case 0x04:
						  for ( k = 0; k < 4; ++k )
							  Vel_Target.byte[3-k]= Rx_Data[k];
						  Speed_TIM6_Target = (int32_t)( Vel_Target.INT /200.0 );
						  //velocity will be measured in CNT/5sec -> to get rpm, mutliply by 60/5120
						  if(Speed_TIM6_Target < 0)
						  {
							  CCW = 1;
						  }
						  else
						  {
							  CCW = 0;
						  }
						  break;
					  }

					  COM_Status = 0;
					  break;

				  case 0x12:
				  case 0x36:
					  //PID initialization
					  //turning data bytes into float numbers
					  for( k = 0; k < 4; ++k )
						  Kp.byte[3-k] = Rx_Data[k];
					  vKp = Kp.flt;

					  for( k = 0; k < 4; ++k )
						  Ki.byte[3-k] = Rx_Data[4 + k];
					  vKi = Ki.flt;

					  for( k = 0; k < 4; ++k )
						  Kd.byte[3-k] = Rx_Data[8 + k];
					  vKd = Kd.flt;

					  for( k = 0; k < 4; ++k )
						  Ka.byte[3-k] = Rx_Data[12 + k];
					  vKa = Ka.flt; //what is vKa used for?

					  COM_Status = 0;
					  break;

				  case 0xD0:
					  //start TIM6 timer and enable it's interrupt + NVIC priority
					  //Why is the timer and interrupt enabled here?
					  HAL_TIM_Base_Start_IT(&htim6);
					  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0); //set NVIC priority
					  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn); //enable interrupt
					  COM_Status = 0;
					  break;
				  case 0xFE:
					  CRC_Val = CalculateCRC16( Rx_Data, Rx_Head[5]*2 );
					  Make_Rply_Head( Rply_Head, ( (Rx_Head[2] & 0xFC) | 0x02 ), 0x01, 0, 1);
					  Rply_Data[0] = ( CRC_Val >> 8 ) & 0x00FF;
					  Rply_Data[1] = CRC_Val & 0x00FF;
					  /*CRC_Val = CalculateCRC16( Rply_Data, Rply_Data[5]*2 );
					  Rply_Data[2] = ( CRC_Val >> 8 ) & 0x00FF;
					  Rply_Data[3] = CRC_Val & 0x00FF;*/

					  /*for ( k = 0; k < 8; ++k )
						  Tx_Buffer[k] = Rply_Head[k];
					  for ( k = 0; k < 4; ++k )
						  Tx_Buffer[ k + 8 ] = Rply_Data[k];*/

					  Transmit_DMA( &huart1, Rply_Data, 2 );

					  COM_Status = 0;
					  break;
				  }
			  }

		  }

		  COM_Status = 0;
	  }
	  //HAL_Delay(1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM15;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 15;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 19999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 3214;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD2_Pin|IN1_Pin|IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin IN1_Pin IN2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|IN1_Pin|IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback( UART_HandleTypeDef *UartHandle )
{
	//HAL_UART_Transmit (&huart1, "WHAT\r\n", 5, 100 );
	uartTxDone = 1; //signal that Transfer is cplt

}

void Transmit_DMA(UART_HandleTypeDef *UartHandle, uint8_t* aTxBuffer, uint16_t TXBUFFERSIZE)
{
	HAL_StatusTypeDef sts;

	uartTxDone = 0;
	sts = HAL_UART_Transmit_DMA(UartHandle, (uint8_t*)aTxBuffer, TXBUFFERSIZE);

	if( sts != HAL_OK)
	{
		Error_Handler();
	}
	else
	{
		//wait for uart transfer to be complete
		while (uartTxDone == 0)
		{
		}
	}
}

HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim)
{
	if( htim -> Instance == TIM6 )
	{
		/*
		 let rpm = 60 rev/min
		 = 1 rev / second

		 Encoder reads 64 cts/motor rev * 20 motor/gear rev * 4 * 1 gear rev/s
		 = 5120 cts/sec
		 = 5120 cts/gear rev @ 1rev/sec

		 with a 5 ms interval between encoder reads = 0.005s
		 0.005s * 5120 cts/s = 25.6 cts

		 TIM2 will be reading encoder values

		 speed displayed in revolutions per minute.
		 (# of cts in 0.005s frame) * (1/0.005seconds) * (60seconds/minute) * (1rev/5120cts) = rpm

		 encoder value is set at 32767 (midpoint of 65535) as an offset so that negative counts are easier to deal with.
		 TIM2 0-25 = 65510 which makes getting an opposite rpm harder.

		 first read encoder value
		 Calculate speed
		 reset encoder offset

		 add PID, target speed, and PWM to the motor later.

		 Keep the speed here in cts/0.005seconds
		 * */

		EncVal = TIM2->CNT;
		TIM2->CNT = 32767;

		Speed_TIM6_Measured = (int32_t)( (EncVal - 32767) );
		Err = Speed_TIM6_Target - Speed_TIM6_Measured;

		dU = vKp * (Err - Err_1) + (vKi * Err) + ( vKd * ( Err - 2*Err_1 + Err_2 ) );
		Uk = Uk_1 + dU;
		Uk_1 = Uk;
		Err_2 = Err_1;
		Err_1 = Err;

		//3214/2 =1607
		if( CCW )
		{
			HAL_GPIO_WritePin (GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
			TIM15 -> CCR1 = (uint32_t) ( -1.0 * (Uk/204.0) * 3214.0 );
		}
		else
		{
			HAL_GPIO_WritePin (GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
			HAL_GPIO_WritePin (GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			TIM15 -> CCR1 = (uint32_t) ( (Uk/204.0) * 3214.0 );
		}

		//update the PWM value
	}
}

uint16_t CalculateCRC16( const uint8_t* c_ptr, size_t len )
{
	/*
	 crc_COM = CalculateCRC16(crc_Result, Rx_Head, 6);

	 Rx_Head is an array storing all the bytes

	 * */
	const uint8_t* c = c_ptr; //now storinkg the address of the array that's storing all the bytes
	//*c++ increases the array index? after access the item at index 'c'?? that would be crazy
	accumulator = 0x0000;
	for( len; len > 0; --len )
	{
		x = accumulator << 8 ;
		y = crctable[ ((accumulator >> 8 ) ^ *c)];
		*c++;
		accumulator = x ^ y;
	}
	return accumulator;
}

void Make_Rply_Head(uint8_t* rply_ptr, uint8_t r2, uint8_t r3, uint8_t r4, uint8_t r5)
{
	rply_ptr[0] = 0xA5;
	rply_ptr[1] = 0xFF;
	rply_ptr[2] = r2;
	rply_ptr[3] = r3;
	rply_ptr[4] = r4;
	rply_ptr[5] = r5;
	CRC_Val = CalculateCRC16( rply_ptr, 6 );
	rply_ptr[6] = ( (CRC_Val >> 8) & 0x00FF );
	rply_ptr[7] = ( CRC_Val & 0x00FF );
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
