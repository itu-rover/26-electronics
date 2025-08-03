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
#include "lwip.h"
#include "stddef.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "udpClientRAW.h"
#include "ethernetComHandler.h"
#include "common4defines.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NO_CRC 0
#define CRC_8 1
#define CRC_16 2

#define CRC_TYPE CRC_8


/* Önceden bağlanmış bir cihazın bağlantısı kesildiğinde
 * ne kadar süre daha veri almayı beklesin (saniye/2)
 */
#define IP_Timeout 10
//TIM7 interrupta bknz

#define UART7_BAUDRATE 115200
#define UART7_TX_PIN   8  // PE8
#define UART7_RX_PIN   7  // PE7

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;



/* USER CODE BEGIN PV */
TIM_HandleTypeDef htim7;
uint8_t txBuff[TX_RX_BUFFER_SIZE + CRC_TYPE];
uint8_t rxBuff[TX_RX_BUFFER_SIZE + CRC_TYPE];
uint8_t FLAG_SPI_BUS_IN_USE = 0;
uint8_t ERROR_CODE;
uint8_t rx_buff_crc;




uint8_t ethBuf[ETH_BUFFER];
uint8_t IP_list[10] = {IP_GS, IP_Xavier, IP_Kilicaslan, IP_Kadir, IP_Nano, IP_Yigit, IP_Al, IP_Bakugan, IP_Oguzhan, IP_Emirhan};
uint32_t counterIP;
uint8_t ethernetFlag = 0;
uint8_t connectionTimeout = 0;
uint8_t IPaddress;
uint16_t currentTs;



char uartTransmit[SCIENCE_COMMAND_SIZE];

typedef enum
{
	SPI_CRC_ERROR
}Error_codes_t;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Initialize(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM6_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
static void MX_TIM7_Init(void);
int find_index(int arr[], int size, int target);

void uart7_init(void);
void uart7_gpio_init(void);


uint8_t science_buffer[39];
char caharc;
void uart7_read_bytes(uint8_t *buffer, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++) {
        // RXNE bayrağı set olana kadar bekle
        while (!(UART7->ISR & USART_ISR_RXNE));
        science_buffer[i] = (uint8_t)(UART7->RDR & 0xFF);
    }
}
uint8_t uart7_read_byte(void)
{
    // RX hazır olana kadar bekle
    while (!(UART7->ISR & USART_ISR_RXNE));
    return (uint8_t)(UART7->RDR & 0xFF);
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern struct netif gnetif;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_SPI2_Init();
  MX_LWIP_Init();
  MX_TIM6_Init();
  MX_CRC_Init();

  uart7_init();
  uart7_gpio_init();

  txBuff[9] = 5;
  txBuff[10] = 5;
  txBuff[11] = 5;
  txBuff[12] = 5;
  txBuff[13] = 5;
  txBuff[14] = 5;
  /* USER CODE BEGIN 2 */
  HAL_Delay(2000);
  MX_TIM7_Init();
  //udpServer_init();

  udpClient_connect(6);
  //udpClient_connect(IP_Xavier);

  //tcp_server_init();
  //HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_GPIO_WritePin(GPIOA, NSS_2_Pin, GPIO_PIN_SET);

  //udpClient_connect();


  txBuff[TX_RX_BUFFER_SIZE] = HAL_CRC_Calculate(&hcrc, txBuff, sizeof(txBuff)-CRC_TYPE);
  /* Uyarı verme sebebi CRC_Calc... fonksiyonunu girişleri 32 bit olması lazım	*/
  /* Bir de CRC ayarı değiştirildiğince yine değişmesi gerekiyor, daha düzgün kod yazılması lazım	*/
  /* Ondan önce strncpy gibi kopyalama fonksiyonu eklenmesi lazım -dayanıklı-	*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  MX_LWIP_Process();

	  extern uint16_t last_ts;
	  extern volatile uint8_t failsafe_triggered;
	  extern uint32_t last_msg_tick;
	  uint8_t default_driveWheel[9] = {'1','0','0','0','1','0','0','0','0'};
	  uint8_t default_roboticArm[6] = {'5','5','5','5','5','5'};

	  /* timeStamp sabit kaldığı süreyi hesaplama	*/
	  if(last_ts == currentTs && failsafe_triggered == 0){
		  failsafe_triggered = 1;
	      last_msg_tick = HAL_GetTick();

	  }
	  else if(last_ts != currentTs){
		  last_ts = currentTs;
	      failsafe_triggered = 0;
	  }


	  /* Safety Critic Kod Parçası - Ahmet Efe agamdan benzetim	*/
	  if ((HAL_GetTick() - last_msg_tick > 1000) && (failsafe_triggered == 1)) {
		  memcpy(txBuff, default_driveWheel, DRIVETRAIN_COMMAND_SIZE);
		  memcpy(txBuff + DRIVETRAIN_COMMAND_SIZE, default_roboticArm, ARM_COMMAND_SIZE);
	  }
	  /* Her güzel kod parçasının bir sonu vardır */

		/*while (!(UART7->ISR & USART_ISR_TXE)); // TX boş değilse bekle
		UART7->TDR = 48;
		while (!(UART7->ISR & USART_ISR_TXE)); // TX boş değilse bekle
		UART7->TDR = 49;
		while (!(UART7->ISR & USART_ISR_TXE)); // TX boş değilse bekle
		UART7->TDR = 48;
		while (!(UART7->ISR & USART_ISR_TXE)); // TX boş değilse bekle
		UART7->TDR = 48;
		uart7_read_bytes(science_buffer, 39);
		caharc = uart7_read_byte();*/

	  /*void uart7_test_loopback(void)
	  {
	      uint8_t ch = 'A';

	      // Gönder
	      UART7->TDR = ch;

	      // Gerçek gönderim bitene kadar bekle (TC = Transmission Complete)
	      while (!(UART7->ISR & USART_ISR_TC));

	      // RXNE bayrağı set olana kadar bekle (veri geldiyse)
	      while (!(UART7->ISR & USART_ISR_RXNE));

	      // Gelen veriyi oku
	      uint8_t rx = UART7->RDR;

	      // Kontrol et
	      if (rx == ch) {
	          science_buffer[0] = 10;
	      } else {
	          science_buffer[0] = 99; // hata kodu
	      }
	  }*/





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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim1.Init.Prescaler = 12000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim6.Init.Prescaler = 107;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = (20000)-1;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Başlatma fonksiyonu
  * @param Yok
  * @retval Yok
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Portlarına saat sinyalini aktif etme
   * Gereksiz Portlar Yorum Satırına Alınabilir */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|NSS_2_Pin|LD3_Pin|LD2_Pin
                          |NSS_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin NSS_2_Pin LD3_Pin LD2_Pin
                           NSS_1_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|NSS_2_Pin|LD3_Pin|LD2_Pin
                          |NSS_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);


  /*Configure GPIO pin : LED Pinleri*/

  //PD12 PD13 PB4 LED pinleri
  GPIO_InitStruct.Pin = 0x1 << 12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = 0x1 << 13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = 0x1 << 4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/*	108MHz Timer Girişi
 *  Timer 7'nin amacı, IP arama sırasında geri bildirim
 *  gelme süresini bekleyecek süreyi ayarlamak	*/
static void MX_TIM7_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  __HAL_RCC_TIM7_CLK_ENABLE();

  uint8_t frekans = 2;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 108*5*100-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000-1/frekans;//2000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
  {
	  Error_Handler();
  }

  HAL_NVIC_SetPriority(TIM7_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);

}



void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    //Release NSS after transfer
	HAL_GPIO_WritePin(GPIOA, NSS_2_Pin, GPIO_PIN_SET);
    /*for (int i = 0; i < sizeof(txBuff)-CRC_TYPE; i++) {
        txBuff[i] += 1;
    }*/
    HAL_GPIO_TogglePin(GPIOB, LD3_Pin); //Indicator led

    uint8_t tx_buff_crc;
    tx_buff_crc = HAL_CRC_Calculate(&hcrc, txBuff, sizeof(txBuff)-CRC_TYPE);

    /* 1 byte crc var gibi yazılı aşağısı !! */
    txBuff[TX_RX_BUFFER_SIZE] = tx_buff_crc;
    rx_buff_crc = HAL_CRC_Calculate(&hcrc, rxBuff, sizeof(rxBuff)-CRC_TYPE);

    if(rx_buff_crc != rxBuff[20])
    {
    	ERROR_CODE = SPI_CRC_ERROR;
    	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    }
    FLAG_SPI_BUS_IN_USE = 0;


}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//

	if(htim->Instance == (htim6.Instance) && !FLAG_SPI_BUS_IN_USE)
	{
		HAL_GPIO_WritePin(GPIOB, NSS_2_Pin, GPIO_PIN_RESET);
		if(HAL_SPI_TransmitReceive_DMA(&hspi2, txBuff,rxBuff, sizeof(txBuff)) != HAL_OK)
		{
			Error_Handler();
		}
		FLAG_SPI_BUS_IN_USE = 1;
	}
	if(htim->Instance == (htim7.Instance))
	{

	}
	else if(htim == &htim1)
	{
		//udpClient_send();
	}

}

/*	IP arama için kullanılan timer ISR fonksiyonu	*/
void TIM7_IRQHandler(void)
{
    if (__HAL_TIM_GET_IT_SOURCE(&htim7, TIM_IT_UPDATE) != RESET)
    {
        __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);


        //udpClient_connect(*(IP_list + counterIP));

        connectionTimeout ++;
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        if (connectionTimeout <= IP_Timeout && ethernetFlag == 1)
        {
        	//find_index( (int)IP_list, sizeof(IP_list), IPaddress);
        	HAL_Delay(10);//udpClient_connect(*(IP_list + counterIP));
        	return;
        }
        else
        {
        	connectionTimeout = 0;

        	ethernetFlag = 0; //Geçici süreliğine,
        	/* Bağlanmı bir cihaz ile bağlantı giderse
        	 * diğer cihazları aramayı iptal ettim
        	 */
        }

        if (ethernetFlag == 0)
        {
        udpClient_connect(*(IP_list + counterIP));
        //HAL_Delay(1000);
        counterIP = (counterIP + 1) % sizeof(IP_list);
        }
    }
}
void IP_finder()
{
	connectionTimeout ++;
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	if (connectionTimeout <= IP_Timeout && ethernetFlag == 1)
	{
		//find_index( (int)IP_list, sizeof(IP_list), IPaddress);
	    HAL_Delay(10);
	    udpClient_connect(*(IP_list + counterIP));
	    return;
    }
	else
	{
		connectionTimeout = 0;

	    ethernetFlag = 0; //Geçici süreliğine,
	    /* Bağlanmı bir cihaz ile bağlantı giderse
	    * diğer cihazları aramayı iptal ettim
	    */ //düzeltildi
	    }

	        if (ethernetFlag == 0)
	        {
	        udpClient_connect(*(IP_list + counterIP));
	        //HAL_Delay(1000);
	        counterIP = (counterIP + 1) % sizeof(IP_list);
	        }
}

int find_index(int arr[], int size, int target) {
    for (int i = 0; i < size; i++) {
        if (arr[i] == target) {
            return i;
        }
    }
    return -1;
}




void uart7_gpio_init(void)
{
    //GPIOE saatini aç
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

    //PE7 ve PE8 -> Alternate Function
    GPIOE->MODER &= ~((3 << (2 * UART7_TX_PIN)) | (3 << (2 * UART7_RX_PIN)));
    GPIOE->MODER |=  ((2 << (2 * UART7_TX_PIN)) | (2 << (2 * UART7_RX_PIN)));

    //Push-pull, high speed
    GPIOE->OSPEEDR |= (3 << (2 * UART7_TX_PIN)) | (3 << (2 * UART7_RX_PIN));
    GPIOE->OTYPER &= ~((1 << UART7_TX_PIN) | (1 << UART7_RX_PIN));
    GPIOE->PUPDR  &= ~((3 << (2 * UART7_TX_PIN)) | (3 << (2 * UART7_RX_PIN)));

    //Alternate Function 8 (UART7) için AFRL/AFRH ayarları
    GPIOE->AFR[1] &= ~((0xF << ((UART7_TX_PIN - 8) * 4)) | (0xF << ((UART7_RX_PIN - 8) * 4)));
    GPIOE->AFR[1] |=  ((8 << ((UART7_TX_PIN - 8) * 4)) | (8 << ((UART7_RX_PIN - 8) * 4)));
}

void uart7_init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_UART7EN;


    //USART disable önce
    UART7->CR1 &= ~USART_CR1_UE;

    // Baudrate ayarı (APB1 Clock varsayılan olarak 54 MHz kabul edildi)
    // USARTDIV = fck / baud => 54000000 / 115200 = 468.75
    // Mantıksal olarak 469 yazılabilir. -- 235 de oldu nedense. Osiloskoptaban bakınca 57600 gibi gözüktü baudrate, ikiye böldüm
    UART7->BRR = 235;

    // 8N1, no parity, oversampling 16x default
    UART7->CR1 = USART_CR1_TE | USART_CR1_RE;  // TX ve RX aktif
    UART7->CR1 |= USART_CR1_UE;                // USART'ı enable et

    // Hazır olana kadar bekle
    while (!(UART7->ISR & USART_ISR_TEACK));
    while (!(UART7->ISR & USART_ISR_REACK));
}



/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x20040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
