/* **************************ENDUSTRIYEL ELEKTRIK***************************** */
/* *************************ENGINEER:IBRAHIM GURAN**************************** */
/* Not: Bu kod NUCLEO F411 kartı üzerinde verici pozisyondayken denenmiş ve
 * çalışmıştır. Verici bir kart olarak yalnızca "ping" verisini alıcıya gönderir.
 * Bu veri gönderme işlemini değiştirebilir veya alınca vereceği cevabı
 * değiştirerek farklı uygulamalar yapabilirsiniz. Kod üzerinde anlaşılması güç
 * noktaları da yorum satırlarıyla anlatacağım. Bu projede bana destek olan ve
 * bana bu proje üzerinde çalışmama izin veren başta HIKMET ALKILINÇ ve YUSUF OZYER
 * olmak üzere ENDUSTRIYEL ELEKTRIK ARGE MERKEZİ mühendislerine ve doğrudan bağlantı
 * kuramasam da kütüphane oluşturmama yardım eden WARD ALMASARANI'ye teşekkür ederim.
 */
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
#include "cmsis_os.h" // Konfigürasyonda freertosla gelen bir kütüphane

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"		// task kullanmamın sebebi UC'ün yükünü azaltmak ve karışıklığı engellemek, led yakma uygulamasında kullanmayabilirsiniz.
#include "e22900t22d.h" // E22-900T22X modülünün kütüphanesi
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
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1; //modüle uart üzerinden veri gönderiyoruz.
DMA_HandleTypeDef hdma_usart1_rx; // USART1 kullanmamın sebebi USART2'de STLINK'in olması.
DMA_HandleTypeDef hdma_usart1_tx; // USART2'yi açsam bile TX pinlere gitmediği için kullanamıyorum.

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM10_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
// Burada lora modülünün çalışması için önemli olan fonksiyonlar bulunmakta.
// Hepsini fonksiyon içerisinde açıklayacağım. Burada sadece gerekli olan tanımlama işlemi var.
static void led_toggle_task(void *parameter);
static void e22_handle_task(void *parameter);
static void e22_transmission_task(void *parameter);
static void main_e22_transceiverMode(void);
static void main_e22_configurationMode(void);
static void main_lora_packet_receive(uint8_t* dataPacket, uint8_t size);
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
  MX_USART1_UART_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  // E22 LoRa modülünü yapılandırıyoruz.
  e22_lora_init(&huart1,
		  	  	HAL_UART_Transmit_DMA,
				HAL_UARTEx_ReceiveToIdle_DMA,
				main_lora_packet_receive,
				main_e22_configurationMode,
				main_e22_transceiverMode);

  // FREERTOS Task oluşturmayı başlatıyoruz.
  xTaskCreate(led_toggle_task, "Toggle GPIO13", 128, NULL, 1, NULL);
  xTaskCreate(e22_handle_task, "E22 LoRa Handler", 128 * 4, NULL, 1, NULL);
  xTaskCreate(e22_transmission_task, "E22 LoRa Tx Task", 128 * 4, NULL, 1, NULL);

  vTaskStartScheduler();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

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
  if (HAL_TIM_IC_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim10, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M0_Pin|M1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOARD_LED_Pin */
  GPIO_InitStruct.Pin = BOARD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOARD_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M0_Pin M1_Pin */
  GPIO_InitStruct.Pin = M0_Pin|M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void led_toggle_task(void *parameter)
{
  static TickType_t xLastWakeTime; //görevin periyodik olarak çalışmasını sağlamak için
  const TickType_t xPeriod = pdMS_TO_TICKS(1000);  // 1 saniye delay
  //pdMS_TO_TICKS makrosu, milisaniye cinsinden verilen değeri FreeRTOS'un tick sayısına dönüştürür.​
  xLastWakeTime = xTaskGetTickCount();
  //Geçerli tick sayısı alınarak xLastWakeTime değişkenine atanıyor. Bu, görevin doğru periyotlarla
  //çalışmasını sağlamak için başlangıç referansı olarak kullanılır.

  for(;;) //sonsuz döngü
  {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
    //Görev, xLastWakeTime zamanından itibaren xPeriod süresi kadar bekler.
    //Bu fonksiyon, görevin kesin periyotlarla çalışmasını sağlar ve
    //zamanlamadaki kaymaları önler.
   // HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
  }
}

// E22 LoRa modülünün çalışma döngüsünü kontrol etmek için yapılmış bir background task’tır.
static void e22_handle_task(void *parameter)
{
  static TickType_t xLastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS(100);
  xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
    // E22 LoRa modülünün yönetim fonksiyonu çağrılır.
    // Bu fonksiyon, modülün durumunu kontrol eder ve gerekli işlemleri yapar.
    e22_lora_manager();
  }
}

static void e22_transmission_task(void *parameter)
{
  static TickType_t xLastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS(1000);
  xLastWakeTime = xTaskGetTickCount();

  const uint8_t packetSize = 4; //  Gönderilecek paketin boyutu
  const uint8_t receiverAddress = 0x09; // Alıcının adresi
  const uint8_t ComChannel = 0x06; //  İletişim kanalı
  uint8_t packet[5] = "ping"; // Gönderilecek veri paketi

  for(;;)
  {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
    e22_lora_transnit(packet, packetSize, receiverAddress, ComChannel);
    // iletimi başlat, iletimdeyken ledi toggle yap
    HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
  }
}

static void main_e22_transceiverMode(void)
{
  // F411'de M0 ve M1 pinleri kullanılmıyor
}

static void main_e22_configurationMode(void)
{
  // F411'de M0 ve M1 pinleri kullanılmıyor
}
// UART ile bir veri gönderme işlemi tamamlandığında otomatik olarak çağrılır.
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  e22_lora_make_ready();
// Bu fonksiyon, E22 modülünü yeni bir işleme (örneğin başka bir paket gönderimi)
// hazır duruma getirmek için çağrılır.
// Gönderim tamamlandıktan sonra sistemin yeni işlem yapabilmesi
// için tetikleme yapılmış olur.

}
// UART üzerinden veri alımı tamamlandığında otomatik olarak
// çağrılan bir HAL fonksiyonudur (DMA veya kesmeyle çalıştığında kullanılır).
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  e22_lora_reception_complete(Size);
// Bu fonksiyon, alınan verinin işlenmesini başlatır.
// Alınan Size kadar byte’lık veriyi ele alacak şekilde E22 modülünün
// işleyici fonksiyonuna yönlendirir.
}

// Bu fonksiyon, gelen LoRa paketlerini yorumlamak için tasarlanmıştır.
// Veriyi alır, içerik kontrolü yapar ve belirli komutlara karşı tepki verir.
static void main_lora_packet_receive(uint8_t* dataPacket, uint8_t size)
{
  uint8_t loraPacket[MAX_DATA_PACKET_SIZE] = {0}; // Gelen paketi tutmak için sıfırlarla başlatılmış bir tampon (buffer) oluşturuluyor.
  memcpy(&loraPacket, dataPacket, size);
 //Gelen veri (dataPacket) size kadar loraPacket tamponuna kopyalanıyor.

  if (0 == memcmp(loraPacket, "pong", size))
  {
	  // Kopyalanan veri ile "pong" kelimesi aynı mı diye karşılaştırma yapılır.
	  // memcmp sonucu 0 ise iki veri aynıdır.
    HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
    // Eğer gelen veri "pong" ise, bir LED’in durumu değiştirilir (açıkken kapanır, kapalıysa açılır).
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
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
