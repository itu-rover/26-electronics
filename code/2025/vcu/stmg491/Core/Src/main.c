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
#include "vesc_can_com.h"
#include "string.h"
#include "stdlib.h"
#include "libft.h"


#include "common4defines.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/*
 * DEBUG = 1 CAN Deaktif
 * DEBUG = 0 CAN Aktif
 */
#define DEBUG_AY 0
#define DEBUG_RK 1


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;


/* 	Alt Yürür	*/
FDCAN_HandleTypeDef hfdcan1;

/*	Robot Kol	*/
FDCAN_HandleTypeDef hfdcan2;



SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */


uint8_t spi_txBuff[SPI_DATA_SIZE+1];
uint8_t spi_rxBuff[SPI_DATA_SIZE+1];// = {0};

//spi_txBuff[DRIVETRAIN_FEEDBACK_SIZE + ARM_FEEDBACK_SIZE]

FDCAN_TxHeaderTypeDef TxHeader;
volatile FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_FilterTypeDef RxFilter;
uint8_t run = 0;
uint8_t               TxData[8];
volatile uint8_t               RxData[8];
int32_t speed = 0;
int32_t angular_speed = 0;
int32_t speed_data[4] = {0};
int32_t speed_data_tmp[4] = {0};
uint32_t TxMailbox;
uint8_t s_pos = 0;
uint8_t rx_count = 0;
int alicisayac;
uint8_t tx_data[34] = {0};
uint8_t s_flag = 0;
uint8_t msg_length = 0;
uint8_t can_send_msg = 0;
int32_t axis[6] = {0};
uint8_t buffer[50] = {0};
uint32_t ID = 0;
int32_t speed_r = 0;
int32_t speed_l = 0;
int i = 0;
uint8_t led_state = 0;



//float Kp = 12;
//int32_t absolute_pos[6] ={0};
//int32_t fixed_pos[6] = {0};
//int32_t absolute_ref_tmp[6] = {0};


//GNSS Global Variables
uint8_t gnss_rxBuff[100];
uint8_t gnss_parsedRxData[18] = {0};
GNSS_Data_t GNSS_data;

//
uint8_t enable[6] = {0};
uint32_t IDs[4] = {0};
uint16_t pos[6] = {0};
uint16_t pos_old[6] = {0};
int32_t turn[6] = {0};
uint32_t TxMailbox;
int32_t ref[6] = {0};
uint32_t data_number =0;


uint8_t rxgnssdata[128];
uint8_t vel_control = 1;
int32_t base_turn = 0;
int32_t base_offset = 0;
int32_t speed_tmp[6] = {0};
uint8_t gripper_speed = 5;
int32_t absolute_pos_for_speed[6] = {0};
uint8_t enable_pos_for_speed[6] = {0};



/*
 * Robot Kol değişkenleri, Ahmet Efe
 */

/*
 * 103.18 * gear1 (16.67) = 1720
 * 143.31 * gear2 (41) = 5875.71
 * 126.11 * gear3 (50) = 6300.55
 * 180 * gear4 (28) = 5040
 * 183.44 * gear5 (22.27) = 4085.22
 * 360 * gear6 (7.8) = 2808
 */


//int i = 0;

FDCAN_TxHeaderTypeDef TxHeaderRK;
volatile FDCAN_RxHeaderTypeDef RxHeaderRK;
FDCAN_FilterTypeDef RxFilterRK;


float Kp = 12;
int32_t absolute_pos[6] ={0};
int32_t fixed_pos[6] = {0};
int32_t absolute_ref_tmp[6] = {0};

volatile uint8_t velocity_flag = 0;
volatile uint8_t position_flag = 0;
volatile uint32_t last_msg_tick = 0;
volatile uint8_t failsafe_triggered = 0;
static uint32_t velocity_tick = 0;
static uint32_t position_tick = 0;
uint32_t last_tick = 0;
uint8_t current_motor = 0;

int32_t speedRK[6] = {0};


/*Silinecek*/
uint8_t mydata[100];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* Robot Kol Fonksiyonları Prototipleri	*/


/* CAN Bağlantısı Prototipi	*/
static void MX_FDCAN2_Init(void);

/* Motor Kontrol Fonksiyonları Prototipleri	*/
void HandleVelocityControl(uint8_t motor_idx);
void HandlePositionControl(uint8_t motor_idx);
void TransmitPosition();
int16_t speed_calc(uint8_t speed);
int32_t map_value(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);


/*	F7-G4 Arası Bağlantı Fonksiyonu Prototipi	*/
static void process_spi_msg();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t bin2hex(uint32_t num) {
	uint8_t hex_value = 0;
	if (num >= 0 && num <= 9) {
		hex_value = '0' + num;
	}
	else if (num >= 10 && num < 16) {
		hex_value = 'A' + num - 10;
	}
	return hex_value;
}


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
  MX_FDCAN1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  MX_FDCAN2_Init();


  /* Alt Yürür CAN Header Ayarları */
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.Identifier = 0x0233;
  TxHeader.IdType = FDCAN_EXTENDED_ID;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
/*
  RxFilter.IdType = FDCAN_EXTENDED_ID;
  RxFilter.FilterIndex = 0;
  RxFilter.FilterType = FDCAN_FILTER_MASK;
  RxFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  RxFilter.FilterID1 = 0x000;
  RxFilter.FilterID2 = 0x000;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &RxFilter) != HAL_OK)
  {
    Error_Handler();
  }
*/


  /* Robot Kol CAN Header Ayarları	*/
  TxHeaderRK.DataLength = FDCAN_DLC_BYTES_8;
  TxHeaderRK.TxFrameType = FDCAN_DATA_FRAME;
  TxHeaderRK.Identifier = 0x0233;
  TxHeaderRK.IdType = FDCAN_EXTENDED_ID;
  TxHeaderRK.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeaderRK.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeaderRK.MessageMarker = 0;
  TxHeaderRK.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeaderRK.ErrorStateIndicator = FDCAN_ESI_PASSIVE;


  if(HAL_SPI_TransmitReceive_DMA(&hspi1, spi_txBuff, spi_rxBuff, sizeof(spi_txBuff)) != HAL_OK)
  {
	  Error_Handler();
  }

  HAL_Delay(500);


  #if !DEBUG_AY

  HAL_FDCAN_Start(&hfdcan1);

  /* FDCAN1 Alıcı Interrupt Aktifleştirme*/
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
	  Error_Handler();
  }

  #endif



  #if !DEBUG_RK

  HAL_FDCAN_Start(&hfdcan2);


  /* FDCAN2 Alıcı Interrupt Aktifleştirme*/
  if(HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
	  Error_Handler();
  }

  VESC_RUN_HANDBRAKE(0x32, &TxHeaderRK, &hfdcan2);

  #endif


  //HAL_TIM_Base_Start_IT(&htim3);
  /*HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim4);*/
  HAL_UART_Receive_DMA(&huart3, gnss_rxBuff,1);
  //uartDmaOffset+=48;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	 //HAL_UART_Receive(&huart3, mydata, 10, HAL_MAX_DELAY);
#if !DEBUG_AY
	  if((speed_r > 1400) || (speed_r < -1400)){
	  			  VESC_RUN_RPM(speed_r, 0x29, &TxHeader, &hfdcan1);
	  			  HAL_Delay(20);
	  			  VESC_RUN_RPM(speed_r, 0x2C, &TxHeader, &hfdcan1);
	  		  }
	  		  else{
	  			  VESC_RUN_HANDBRAKE(0x29, &TxHeader, &hfdcan1);
	  			  HAL_Delay(20);
	  			  VESC_RUN_HANDBRAKE(0x2C, &TxHeader, &hfdcan1);
	  		  }
	  		  HAL_Delay(20);
	  		  if((speed_l > 1400) || (speed_l < -1400)){
	  			  VESC_RUN_RPM(speed_l, 0x23, &TxHeader, &hfdcan1);
	  			  HAL_Delay(20);
	  			  VESC_RUN_RPM(speed_l, 0x24, &TxHeader, &hfdcan1);
	  		  }
	  		  else{
	  			  VESC_RUN_HANDBRAKE(0x23, &TxHeader, &hfdcan1);
	  			  HAL_Delay(20);
	  			  VESC_RUN_HANDBRAKE(0x24, &TxHeader, &hfdcan1);
	  		  }

	  HAL_Delay(20);
#endif


#if !DEBUG_RK
	  /*  Robot Kol Ahmet Efe'nin kod parçası	*/

	  if ((HAL_GetTick() - last_msg_tick > 500) && (failsafe_triggered == 0)) {
	          for (uint8_t i = 0; i < 6; i++) {
	              speedRK[i] = 5;
	          }
	          vel_control = 1;
	          failsafe_triggered = 1;
	      }

	      static uint32_t last_tick = 0;
	      static uint8_t current_motor = 0;
	      uint32_t now = HAL_GetTick();

	      if (now - last_tick >= 5) {//her 5msde bir motor
	          if (vel_control == 1)
	              HandleVelocityControl(current_motor);
	          else
	              HandlePositionControl(current_motor);

	          //6 motor tamamlanınca pozisyon verisini yolla
	          if (current_motor == 5) {
	              TransmitPosition();
	          }

	          current_motor = (current_motor + 1) % 6;//döngü
	          last_tick = now;
	      }
	      HAL_Delay(1);
#endif

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 56;
  hfdcan1.Init.NominalTimeSeg2 = 28;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

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
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 339;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499999;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 680;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 6800;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 460800;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LPUART1_TX_Pin LPUART1_RX_Pin */
  GPIO_InitStruct.Pin = LPUART1_TX_Pin|LPUART1_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_LPUART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/*void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	if((RxFifo0ITs && FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET){
				if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){
					Error_Handler();
				}
				if ((RxHeader.Identifier >> 8) == 0x9){
					ID = RxHeader.Identifier & 0xFF;
					switch (ID){
					case 0x2C:
						speed_data[0] = (RxData[0] << 24) | (RxData[1] << 16) | (RxData[2] << 8) | RxData[3];
						break;
					case 0x29:
						speed_data[1] = (RxData[0] << 24) | (RxData[1] << 16) | (RxData[2] << 8) | RxData[3];
						break;
					case 0x23:
						speed_data[2] = (RxData[0] << 24) | (RxData[1] << 16) | (RxData[2] << 8) | RxData[3];
						break;
					case 0x24:
						speed_data[3] = (RxData[0] << 24) | (RxData[1] << 16) | (RxData[2] << 8) | RxData[3];
						break;
					}

				}
				if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
					Error_Handler();
				}
			}
	if(hfdcan->Instance == FDCAN2) // Sadece FDCAN2 için
		{
		if((RxFifo0ITs && FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET){
						if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){
							Error_Handler();
						}
						if ((RxHeader.Identifier & 0x1000) == 0x1000){
							ID = RxHeader.Identifier & 0xFF;
							switch (ID){
							case 0x33:
								pos[0] = ((RxData[6] << 8) | RxData[7]) / 50;
								if (pos[0] - pos_old[0] > 330) {
									turn[0] -= 1;
								}
								if (pos_old[0] - pos[0] > 330) {
									turn[0] += 1;
								}
								if((speed_tmp[0] != 0) || (vel_control == 0)) absolute_pos[0] = turn[0]*360 + pos[0];
								absolute_pos_for_speed[0] = turn[0]*360 + pos[0];
								pos_old[0] = pos[0];
								if(enable[0] == 0){
									enable[0] = 1;
									absolute_pos[0] = turn[0]*360 + pos[0];
								}
								break;
							case 0x1E:
								pos[1] = ((RxData[6] << 8) | RxData[7]) / 50;
								if (pos[1] - pos_old[1] > 330) {
									turn[1] -= 1;
								}
								if (pos_old[1] - pos[1] > 330) {
									turn[1] += 1;
								}
								if((speed_tmp[1] != 0) || (vel_control == 0)) absolute_pos[1] = turn[1]*360 + pos[1];
								absolute_pos_for_speed[1] = turn[1]*360 + pos[1];
								pos_old[1] = pos[1];
								if(enable[1] == 0){
									enable[1] = 1;
									absolute_pos[1] = turn[1]*360 + pos[1];
								}
								break;
							case 0x77:
								pos[2] = ((RxData[6] << 8) | RxData[7]) / 50;
								if (pos[2] - pos_old[2] > 330) {
									turn[2] -= 1;
								}
								if (pos_old[2] - pos[2] > 330) {
									turn[2] += 1;
								}
								if((speed_tmp[2] != 0) || (vel_control == 0)) absolute_pos[2] = turn[2]*360 + pos[2];
								absolute_pos_for_speed[2] = turn[2]*360 + pos[2];
								pos_old[2] = pos[2];
								if(enable[2] == 0){
									enable[2] = 1;
									absolute_pos[2] = turn[2]*360 + pos[2];
								}
								break;
							case 0x32:
								pos[3] = ((RxData[6] << 8) | RxData[7]) / 50;
								if (pos[3] - pos_old[3] > 330) {
									turn[3] -= 1;
								}
								if (pos_old[3] - pos[3] > 330) {
									turn[3] += 1;
								}
								if((speed_tmp[3] != 0) || (vel_control == 0)) absolute_pos[3] = turn[3]*360 + pos[3];
								absolute_pos_for_speed[3] = turn[3]*360 + pos[3];
								pos_old[3] = pos[3];
								if(enable[3] == 0){
									enable[3] = 1;
									absolute_pos[3] = turn[3]*360 + pos[3];
								}
								break;
							case 0x1C:
								pos[4] = ((RxData[6] << 8) | RxData[7]) / 50;
								if (pos[4] - pos_old[4] > 330) {
									turn[4] -= 1;
								}
								if (pos_old[4] - pos[4] > 330) {
									turn[4] += 1;
								}
								if((speed_tmp[4] != 0) || (vel_control == 0)) absolute_pos[4] = turn[4]*360 + pos[4];
								absolute_pos_for_speed[4] = turn[4]*360 + pos[4];
								pos_old[4] = pos[4];
								if(enable[4] == 0){
									enable[4] = 1;
									absolute_pos[4] = turn[4]*360 + pos[4];
								}
								break;
							case 0x6D:
								pos[5] = ((RxData[6] << 8) | RxData[7]) / 50;
								if (pos[5] - pos_old[5] > 330) {
									turn[5] -= 1;
								}
								if (pos_old[5] - pos[5] > 330) {
									turn[5] += 1;
								}
								if((speed_tmp[5] != 0) || (vel_control == 0)) absolute_pos[5] = turn[5]*360 + pos[5];
								absolute_pos_for_speed[5] = turn[5]*360 + pos[5];
								pos_old[5] = pos[5];
								if(enable[5] == 0){
									enable[5] = 1;
									absolute_pos[5] = turn[5]*360 + pos[5];
								}
								break;
							}

						}
						if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
							Error_Handler();
						}
					}
	    }

}*/


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	if((RxFifo0ITs && FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET){
				if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeaderRK, RxData) != HAL_OK){
					Error_Handler();
				}
				if ((RxHeaderRK.Identifier & 0x1000) == 0x1000){
					ID = RxHeaderRK.Identifier & 0xFF;
					switch (ID){
					case 0x33:
						pos[0] = ((RxData[6] << 8) | RxData[7]) / 50;
						if (pos[0] - pos_old[0] > 330) {
							turn[0] -= 1;
						}
						if (pos_old[0] - pos[0] > 330) {
							turn[0] += 1;
						}
						if((speed_tmp[0] != 0) || (vel_control == 0)) absolute_pos[0] = turn[0]*360 + pos[0];
						absolute_pos_for_speed[0] = turn[0]*360 + pos[0];
						pos_old[0] = pos[0];
						if(enable[0] == 0){
							enable[0] = 1;
							absolute_pos[0] = turn[0]*360 + pos[0];
						}
						break;
					case 0x1E:
						pos[1] = ((RxData[6] << 8) | RxData[7]) / 50;
						if (pos[1] - pos_old[1] > 330) {
							turn[1] -= 1;
						}
						if (pos_old[1] - pos[1] > 330) {
							turn[1] += 1;
						}
						if((speed_tmp[1] != 0) || (vel_control == 0)) absolute_pos[1] = turn[1]*360 + pos[1];
						absolute_pos_for_speed[1] = turn[1]*360 + pos[1];
						pos_old[1] = pos[1];
						if(enable[1] == 0){
							enable[1] = 1;
							absolute_pos[1] = turn[1]*360 + pos[1];
						}
						break;
					case 0x77:
						pos[2] = ((RxData[6] << 8) | RxData[7]) / 50;
						if (pos[2] - pos_old[2] > 330) {
							turn[2] -= 1;
						}
						if (pos_old[2] - pos[2] > 330) {
							turn[2] += 1;
						}
						if((speed_tmp[2] != 0) || (vel_control == 0)) absolute_pos[2] = turn[2]*360 + pos[2];
						absolute_pos_for_speed[2] = turn[2]*360 + pos[2];
						pos_old[2] = pos[2];
						if(enable[2] == 0){
							enable[2] = 1;
							absolute_pos[2] = turn[2]*360 + pos[2];
						}
						break;
					case 0x32:
						pos[3] = ((RxData[6] << 8) | RxData[7]) / 50;
						if (pos[3] - pos_old[3] > 330) {
							turn[3] -= 1;
						}
						if (pos_old[3] - pos[3] > 330) {
							turn[3] += 1;
						}
						if((speed_tmp[3] != 0) || (vel_control == 0)) absolute_pos[3] = turn[3]*360 + pos[3];
						absolute_pos_for_speed[3] = turn[3]*360 + pos[3];
						pos_old[3] = pos[3];
						if(enable[3] == 0){
							enable[3] = 1;
							absolute_pos[3] = turn[3]*360 + pos[3];
						}
						break;
					case 0x1C:
						pos[4] = ((RxData[6] << 8) | RxData[7]) / 50;
						if (pos[4] - pos_old[4] > 330) {
							turn[4] -= 1;
						}
						if (pos_old[4] - pos[4] > 330) {
							turn[4] += 1;
						}
						if((speed_tmp[4] != 0) || (vel_control == 0)) absolute_pos[4] = turn[4]*360 + pos[4];
						absolute_pos_for_speed[4] = turn[4]*360 + pos[4];
						pos_old[4] = pos[4];
						if(enable[4] == 0){
							enable[4] = 1;
							absolute_pos[4] = turn[4]*360 + pos[4];
						}
						break;
					case 0x6D:
						pos[5] = ((RxData[6] << 8) | RxData[7]) / 50;
						if (pos[5] - pos_old[5] > 330) {
							turn[5] -= 1;
						}
						if (pos_old[5] - pos[5] > 330) {
							turn[5] += 1;
						}
						if((speed_tmp[5] != 0) || (vel_control == 0)) absolute_pos[5] = turn[5]*360 + pos[5];
						absolute_pos_for_speed[5] = turn[5]*360 + pos[5];
						pos_old[5] = pos[5];
						if(enable[5] == 0){
							enable[5] = 1;
							absolute_pos[5] = turn[5]*360 + pos[5];
						}
						break;
					}

				}
				if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
					Error_Handler();
				}
			}

}






static void process_spi_msg()
{
	    TIM2->CNT = 0;
	    //int spi_rxBuff_len = sizeof(spi_rxBuff);
	    char temp_data[50] = {0};
	    int temp_idx = 0;
	    int32_t temp_speed, angtemp_speed;

	    for (int i = 0; i < 9; i++) {
	    	temp_data[i] = spi_rxBuff[i];
	    	temp_idx++;
	    }
	    if (temp_idx >= 9) {
	        angtemp_speed = 10 * ((temp_data[1]-48) * 100 + (temp_data[2]-48) * 10 + (temp_data[3]-48));
	        //angular_speed = (temp_data[0]-48) ? angular_speed : -angular_speed;
	        if (temp_data[0] == 49) {
	        	angular_speed = angtemp_speed;
	        } else if (temp_data[0] == 48){
	        	angular_speed = -1 * angtemp_speed;
	        }
	        /*else {
	        	angular_speed = 0;
	        }*/

	        temp_speed = 10 * ((temp_data[5]-48) * 1000 + (temp_data[6]-48) * 100 + (temp_data[7]-48) * 10 + (temp_data[8]-48));
	        //speed = (temp_data[4]-48) ? temp_speed : (-1 * temp_speed);

	        if (temp_data[4] == 49) {
	            speed = temp_speed;
	        } else if (temp_data[4] == 48){
	            speed = -1 * temp_speed;
	        }
	        else {
	        	speed = 0;
	        }

	        speed_r = speed + angular_speed * 10;
	        speed_l = speed - angular_speed * 10;
	    }


	    /*  Robot Kol Komutları	İşleme	*/
	    for(int i = 0; i < ARM_COMMAND_SIZE; i++)
	    {
	    	speedRK[i] = spi_rxBuff[DRIVETRAIN_COMMAND_SIZE+i];
	    }


}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2){
		speed_l = 0;
		speed_r = 0;
	}
	if (htim == &htim4){
		//HAL_UART_Transmit(&huart3, &led_state, 1, 10);
	}
	if (htim == &htim3){


		  for(i = 0;i<4;i++){
			  speed_data_tmp[i] = speed_data[i];
		  }

		  tx_data[41] = 'X';
		  for (i = 0; i < 8; i++){
			  tx_data[8-i] = bin2hex(speed_data_tmp[0] & 0xF);
			  speed_data_tmp[0] = speed_data_tmp[0] >> 4;
		  }
		  for (i = 0; i < 8; i++){
			  tx_data[16-i] = bin2hex(speed_data_tmp[1] & 0xF);
			  speed_data_tmp[1] = speed_data_tmp[1] >> 4;
		  }
		  for (i = 0; i < 8; i++){
			  tx_data[24-i] = bin2hex(speed_data_tmp[2] & 0xF);
			  speed_data_tmp[2] = speed_data_tmp[2] >> 4;
		  }
		  for (i = 0; i < 8; i++){
			  tx_data[32-i] = bin2hex(speed_data_tmp[3] & 0xF);
			  speed_data_tmp[3] = speed_data_tmp[3] >> 4;
		  }
		  /*for (i = 0; i < 8; i++){
			  tx_data[40-i] = '0';
		  }
		  tx_data[0] = 'S';*/

		  ft_memcpy((char *)(spi_txBuff), (char *)tx_data + 1, 32);

		  //HAL_UART_Transmit(&hlpuart1, tx_data, 42, 50);
	}
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{

    spi_txBuff[SPI_DATA_SIZE] = HAL_CRC_Calculate(&hcrc, (uint32_t*)spi_txBuff, sizeof(spi_txBuff)-1);
    if(1) //insert check crc code
    {
    	process_spi_msg();
    }

	/*if(HAL_SPI_TransmitReceive_DMA(&hspi1,spi_txBuff, spi_rxBuff,sizeof(spi_txBuff)) != HAL_OK)
	{
		//Error_Handler();
	}*/


}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (GNSS_PVTmsgSync(gnss_rxBuff) == 0)
		return;
	GNSS_PVTmsgParse(gnss_rxBuff, gnss_parsedRxData);
	ft_memcpy((char*) (spi_txBuff + DRIVETRAIN_FEEDBACK_SIZE + ARM_FEEDBACK_SIZE),(char*) gnss_parsedRxData , 16);
	GNSS_PVTmsgDisplay(gnss_rxBuff,&GNSS_data);
	HAL_UART_Receive_DMA(&huart3, gnss_rxBuff, 100);
}


/**
  * @brief Robot Kol CAN 2 Başlatma Fonksiyonu
  * @param Yok
  * @retval Yok
  */
static void MX_FDCAN2_Init(void)
{
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 4;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 56;
  hfdcan2.Init.NominalTimeSeg2 = 28;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_QUEUE_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }

}



/*	Robot Kol Fonksiyonları	*/
void TransmitPosition() {
    tx_data[0] = 'S';
    absolute_ref_tmp[0] = map_value(absolute_pos[0], -3000, 3000, 0, 9999);
    absolute_ref_tmp[1] = map_value(absolute_pos[1], -5875, 5875, 0, 9999);
    absolute_ref_tmp[2] = map_value(absolute_pos[2], -6300, 6300, 0, 9999);
    absolute_ref_tmp[3] = map_value(absolute_pos[3], -5040, 5040, 0, 9999);
    absolute_ref_tmp[4] = map_value(absolute_pos[4], -4085, 4085, 0, 9999);
    absolute_ref_tmp[5] = map_value(absolute_pos[5], -2808, 2808, 0, 9999);

    tx_data[30] = 'X';
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 4; j++) {
            tx_data[(4 * (i + 1)) - j] = bin2hex(absolute_ref_tmp[i] & 0xF);
            absolute_ref_tmp[i] >>= 4;
        }
    }
    for (int i = 0; i < 5; i++) {
        tx_data[29 - i] = '0';
    }
    //HAL_UART_Transmit(&hlpuart1, tx_data, 31, 50);
}

void HandleVelocityControl(uint8_t motor_idx) {
    static const uint8_t can_ids[6] = {0x33, 0x1E, 0x77, 0x32, 0x1C, 0x6D};
    int32_t tmp_speed;
    switch (motor_idx) {
        case 0: tmp_speed = speed_calc(speedRK[0]) * 0.8; break;
        case 1: tmp_speed = speed_calc(speedRK[1]); break;
        case 2: tmp_speed = speed_calc(speedRK[2]) * 1.5; break;
        default: tmp_speed = speed_calc(speedRK[motor_idx]); break; //4,5,6 eksenlerinin hız çarpanı yok
    }

    speed_tmp[motor_idx] = tmp_speed;

    if (tmp_speed == 0) {
        if (enable[motor_idx]) {
            if (enable_pos_for_speed[motor_idx]) {
                absolute_pos[motor_idx] = absolute_pos_for_speed[motor_idx];
                enable_pos_for_speed[motor_idx] = 0;
            }
            VESC_RUN_POS(absolute_pos[motor_idx] % 360, can_ids[motor_idx], &TxHeaderRK, &hfdcan2);
        }
    } else {
        if (motor_idx == 1) tmp_speed *= -1;
        if (motor_idx == 3) tmp_speed *= 1.5;
        VESC_RUN_RPM(tmp_speed, can_ids[motor_idx], &TxHeaderRK, &hfdcan2);
        enable_pos_for_speed[motor_idx] = 1;
    }
}

void HandlePositionControl(uint8_t motor_idx) {
    const uint8_t can_ids[6] = {0x33, 0x1E, 0x77, 0x32, 0x1C, 0x6D};
    const int thresholds[6] = {150, 200, 500, 200, 250, 150};
    const int limits[6] = {900, 900, 900, 1500, 900, 900};
    const int gains[6] = {Kp, Kp, Kp, 8, Kp, Kp};

    speedRK[motor_idx] = (ref[motor_idx] - absolute_pos[motor_idx]) * gains[motor_idx];

    if (enable[motor_idx]) {
        if (speedRK[motor_idx] > thresholds[motor_idx] || speedRK[motor_idx] < -thresholds[motor_idx]) {
            if (speedRK[motor_idx] > limits[motor_idx]) speedRK[motor_idx] = limits[motor_idx];
            if (speedRK[motor_idx] < -limits[motor_idx]) speedRK[motor_idx] = -limits[motor_idx];
            VESC_RUN_RPM(speedRK[motor_idx], can_ids[motor_idx], &TxHeaderRK, &hfdcan2);
        } else {
            VESC_RUN_POS(ref[motor_idx] % 360, can_ids[motor_idx], &TxHeaderRK, &hfdcan2);
        }
    }
}
int16_t speed_calc(uint8_t speed){
	switch (speed){
	case 1:
			return -825;
	case 2:
			return -650;
	case 3:
			return -475;
	case 4:
			return -300;
	case 5:
			return 0;
	case 6:
			return 300;
	case 7:
			return 475;
	case 8:
			return 650;
	case 9:
			return 875;
	}
	return 0;
}


int32_t map_value(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
	int32_t value = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return value;
}


/* Robot Kol Kodlarının Ek Fonksiyonlarının Bitiş Yeri
 * Bunu harici bir C koduna çevirmenizi rica ediyorum.
 */


/**
  * @brief  Hata olması durumunda bu fonksiyon çağrılır
  * @retval Yok
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Fonksiyona girmesi durumunda resetle */
	NVIC_SystemReset();
  //__disable_irq();

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
