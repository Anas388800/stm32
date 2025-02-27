/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "queue.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define QUEUE_LENGTH  5    // Nombre maximal de messages dans la file
#define QUEUE_ITEM_SIZE 64 // Taille maximale d'un message
#define SPI_BUFFER_SIZE 50
#define MESSAGE_LENGTH 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UltrasonicTask */
osThreadId_t UltrasonicTaskHandle;
const osThreadAttr_t UltrasonicTask_attributes = {
  .name = "UltrasonicTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for ServoControlTas */
osThreadId_t ServoControlTasHandle;
const osThreadAttr_t ServoControlTas_attributes = {
  .name = "ServoControlTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for SPITask */
osThreadId_t SPITaskHandle;
const osThreadAttr_t SPITask_attributes = {
  .name = "SPITask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

uint8_t spiRxBuffer[SPI_BUFFER_SIZE];
uint8_t messageBuffer[SPI_BUFFER_SIZE]; // Buffer pour le message reçu
uint8_t messageIndex = 0;  // Index pour construire le message
volatile uint8_t spiDataReady = 0; // Indicateur de réception des données SPI

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;

QueueHandle_t DistanceQueue;
QueueHandle_t SPIQueue;

uint8_t rx_buffer[QUEUE_ITEM_SIZE]; // Tampon temporaire pour la réception SPI
volatile uint8_t rx_index = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);
void StartUltrasonicTask(void *argument);
void StartServoControlTas(void *argument);
void StartSPITask(void *argument);

/* USER CODE BEGIN PFP */
uint8_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void SetServoAngle(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t angle) {
    // Calcul de la largeur d'impulsion (en microsecondes)
    // 1 ms (1000 µs) correspond à 0°, 2 ms (2000 µs) correspond à 180°
    uint32_t pulse_length = 1000 + ((angle * 1000) / 180);
    // Ajuster le rapport cyclique (CCR)
    __HAL_TIM_SET_COMPARE(htim, Channel, pulse_length);

}

void SetBuzzer(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t delay) {

	//uint8_t buz_freq = ((TIM_FREQ/(1000*frequency))-1);
	__HAL_TIM_SET_PRESCALER(&htim3, 83);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,200);
	HAL_Delay(delay);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,5000);
	HAL_Delay(delay);


}

void HCSR04_Trigger(void){
	//UART_Debug("HCSR04_Trigger...\r\n");
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);  // Activ Trig (PA5)
	HAL_Delay(10);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET); // Desactive le Trig
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}
void UART_Debug(const char* message) {
    HAL_UART_Transmit(&huart2, (uint32_t*)message, strlen(message),100);
}
void SPI_Display(){
	char uartMessage[] = "STM32 Ready to receive SPI data via interrupt...\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)uartMessage, strlen(uartMessage), HAL_MAX_DELAY);


		if (HAL_SPI_Receive_IT(&hspi1, spiRxBuffer, 1) != HAL_OK) {
				char errorMsg[] = "Failed to initialize SPI interrupt reception.\r\n";
				HAL_UART_Transmit(&huart2, (uint8_t*)errorMsg, strlen(errorMsg), HAL_MAX_DELAY);
		}

		    while (1) {

		    	// Vérifier si un message SPI complet est prêt
					if (spiDataReady)
					{
						spiDataReady = 0; // Réinitialiser l'indicateur
						HAL_UART_Transmit(&huart2, messageBuffer, strlen((char*)messageBuffer), HAL_MAX_DELAY);
						// Ajouter un retour à la ligne pour l'affichage
						char newline[] = "\r\n";
						HAL_UART_Transmit(&huart2, (uint8_t*)newline, strlen(newline), HAL_MAX_DELAY);
						// Nettoyer le flag d’overrun SPI
						__HAL_SPI_CLEAR_OVRFLAG(&hspi1);

						// Réactiver la réception SPI en interruption
						//HAL_SPI_Receive_IT(&hspi1, spiRxBuffer, 1);
						//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Éteindre la LED
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Éteindre la LED après traitement
		            }
		    }

}

void Clock_config_display(){

	uint32_t sysclk = HAL_RCC_GetSysClockFreq();
	uint32_t hclk = HAL_RCC_GetHCLKFreq();
	uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
	uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();

	UART_Debug("------------------\n");
	char sysclk_msg[50];
	sprintf(sysclk_msg,"sysclk (Hz) :%lu\r\n",sysclk);
	UART_Debug(sysclk_msg);
	char hclk_msg[50];
	sprintf(hclk_msg," hclk (Hz) :%lu\r\n",hclk);
	UART_Debug(hclk_msg);
	char pclk1_msg[50];
	sprintf(pclk1_msg," pclk1 (Hz) :%lu\r\n",pclk1);
	UART_Debug(pclk1_msg);
	char pclk2_msg[50];
	sprintf(pclk2_msg," pclk2 (Hz) :%lu\r\n",pclk2);
	UART_Debug(pclk2_msg);
	UART_Debug("------------------\n");

}

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  Clock_config_display();
  UART_Debug("System Initialization...\r\n");
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  __HAL_SPI_ENABLE_IT(&hspi1, SPI_IT_RXNE);
  // Réception

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  DistanceQueue = xQueueCreate(5,sizeof(int));
  if(DistanceQueue == NULL){
	  // Gestion de l'erreur si la queue n'est pas créée
	  Error_Handler();
	  UART_Debug("Distance Queue not created");
  }
  SPIQueue = xQueueCreate(5,MESSAGE_LENGTH);
  //SPIQueue = xQueueCreate(5, sizeof(char *)); // La queue stocke des pointeurs
  if(SPIQueue == NULL){
	  Error_Handler();
	  UART_Debug("SPI Queue not created");
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of UltrasonicTask */
  UltrasonicTaskHandle = osThreadNew(StartUltrasonicTask, NULL, &UltrasonicTask_attributes);

  /* creation of ServoControlTas */
  ServoControlTasHandle = osThreadNew(StartServoControlTas, NULL, &ServoControlTas_attributes);

  /* creation of SPITask */
  SPITaskHandle = osThreadNew(StartSPITask, NULL, &SPITask_attributes);

  if (HAL_SPI_Receive_IT(&hspi1, spiRxBuffer, 1) != HAL_OK) {
  			char errorMsg[] = "Failed to initialize SPI interrupt reception.\r\n";
  			HAL_UART_Transmit(&huart2, (uint8_t*)errorMsg, strlen(errorMsg), HAL_MAX_DELAY);
  }

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
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
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  uint32_t timer_frequency = HAL_RCC_GetPCLK1Freq() / (htim3.Init.Prescaler + 1);
  char timer_msg[50];
  sprintf(timer_msg,"Fréquence du timer: %lu Hz\n", timer_frequency);
  UART_Debug(timer_msg);
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TRIG_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Mode output push-pull
  GPIO_InitStruct.Pull = GPIO_NOPULL;        // Pas de résistance pull-up ou pull-down
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Vitesse faible pour réduire la consommation
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}

		xQueueSendFromISR(DistanceQueue, &Distance, NULL);
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) { // vérifie si l'interruption concerne bien l'instance SPI1 avant de procéder.


        // Ajouter l'octet reçu au messageBuffer
        uint8_t receivedByte = spiRxBuffer[0]; // on ne recoit qun seul octet à la fois
        if (messageIndex < SPI_BUFFER_SIZE - 1) { // Tant que l'index du tableau est inferieur à la taille max du buffer
            messageBuffer[messageIndex++] = receivedByte; // On rempli le tableau octet par octet "B" + "o" + "n" + "j" etc ...
            // Si un caractère de fin est détecté, marquer le message comme prêt
            if (receivedByte == '\n') {
                spiDataReady = 1;                   // Signaler qu'un message complet a été reçu
                messageBuffer[messageIndex] = '\0'; // Terminer la chaîne
                messageIndex = 0;                   // Réinitialiser l'index pour le prochain message
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // Allumer la LED pour indiquer la réception
                xQueueSendFromISR(SPIQueue, &messageBuffer, NULL);

            }
        }
        // Réactiver la réception SPI pour 1 octet
        HAL_SPI_Receive_IT(hspi, spiRxBuffer, 1); // doit être appelée à chaque interruption pour relancer la réception SPI en mode non bloquant.
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
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartUltrasonicTask */
/**
* @brief Function implementing the UltrasonicTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUltrasonicTask */
void StartUltrasonicTask(void *argument)
{
  /* USER CODE BEGIN StartUltrasonicTask */
  /* Infinite loop */
  int distance = 0;
  uint8_t  buz = 0;
  //HAL_TIM_Base_Start(&htim3);
  UART_Debug("Ultrasonic Task Started...\r\n");
  for(;;)
  {
    osDelay(1);
    HCSR04_Trigger(); // Declanche la pin TRIG
    //UART_Debug("Triggering sensor...\r\n");
    if(xQueueReceive(DistanceQueue, &distance, pdMS_TO_TICKS(500))==pdPASS){
    	//UART_Debug("Queue received");

		if(Distance < 80){
			char msg[50];
			sprintf(msg, "Distance: %d cm\r\n", distance);
			HAL_UART_Transmit(&huart2,(uint8_t*)msg, strlen(msg),HAL_MAX_DELAY);
			buz = map(distance, 5, 80, 100, 900);
			SetBuzzer(&htim3, TIM_CHANNEL_1,buz);
		}


    }
    vTaskDelay(pdMS_TO_TICKS(100));




  }
  /* USER CODE END StartUltrasonicTask */
}

/* USER CODE BEGIN Header_StartServoControlTas */
/**
* @brief Function implementing the ServoControlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServoControlTas */
void StartServoControlTas(void *argument)
{
  /* USER CODE BEGIN StartServoControlTas */
  /* Infinite loop */
  uint32_t adc_value = 0;
  uint8_t angle_servo = 0 ;
  UART_Debug("Servo Control Task Started...\r\n");
  for(;;)
  {
	// Lecture de la valeur ADC
	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK){
		adc_value = HAL_ADC_GetValue(&hadc1);
		/*char adc_value_message[50];
		sprintf(adc_value_message,"Adc value : %d\r\n",adc_value);
		UART_Debug(adc_value_message);
		*/
	}
	// Mapper la valeur ADC(0-4095) en un angle de 0 à 180°C
	angle_servo = map(adc_value, 0,4095,0,250);
	// Definir l'angle du servo
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	SetServoAngle(&htim2, TIM_CHANNEL_1, angle_servo);
	vTaskDelay(pdMS_TO_TICKS(50)); // Pause de 50ms entre les mises à jour
  }
  /* USER CODE END StartServoControlTas */
}

/* USER CODE BEGIN Header_StartSPITask */
/**
* @brief Function implementing the SPITask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSPITask */
void StartSPITask(void *argument)
{
  /* USER CODE BEGIN StartSPITask */
	char receivedMessage[QUEUE_ITEM_SIZE];
	char msg_buz1[] = "Bonjour5\n";
	//const char msg_buz1[] = "Bonjour5";
	char msg_buz2[] = "Aurevoir\n";
	//const char msg_buz2[] = "Aurevoir";
	UART_Debug("start_SPI_Task");

  /* Infinite loop */
  for(;;)
  {
	if(xQueueReceive(SPIQueue, &receivedMessage, pdMS_TO_TICKS(500))==pdPASS){
		HAL_UART_Transmit(&huart2, receivedMessage, strlen((char*)receivedMessage), HAL_MAX_DELAY);
		char newline[] = "\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)newline, strlen(newline), HAL_MAX_DELAY);
		spiDataReady = 0; // Réinitialiser l'indicateur
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		if (strcmp((char*)receivedMessage,msg_buz1)==0){
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,500);
			vTaskDelay(10);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,5000);
		}
		else if(strcmp((char*)receivedMessage,msg_buz2)==0){
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,800);
			vTaskDelay(10);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,5000);
		}
	}
    osDelay(1);
  }
  /* USER CODE END StartSPITask */
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
