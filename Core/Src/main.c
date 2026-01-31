/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Frequenze note musicali (Hz)
#define NOTE_A3  220   // La basso
#define NOTE_Bb3 233   // Si bemolle basso
#define NOTE_B3  247   // Si basso
#define NOTE_C4  262   // Do
#define NOTE_Cs4 277   // Do diesis
#define NOTE_D4  294   // Re
#define NOTE_Eb4 311   // Mi bemolle
#define NOTE_E4  330   // Mi
#define NOTE_F4  349   // Fa
#define NOTE_Fs4 370   // Fa diesis
#define NOTE_G4  392   // Sol
#define NOTE_Gs4 415   // Sol diesis
#define NOTE_Ab4 415   // La bemolle
#define NOTE_A4  440   // La
#define NOTE_Bb4 466   // Si bemolle
#define NOTE_B4  494   // Si
#define NOTE_C5  523   // Do alto
#define NOTE_Cs5 554   // Do diesis alto
#define NOTE_D5  587   // Re alto
#define NOTE_Eb5 622   // Mi bemolle alto
#define NOTE_E5  659   // Mi alto
#define NOTE_F5  698   // Fa alto
#define NOTE_G3  196   // Sol basso
#define NOTE_D3  147   // Re basso
#define NOTE_REST 0    // Pausa

// 74HC595 Shift Register - alias per i pin
#define SHIFT_DATA_Pin      RGB_RED_Pin       // PE9 -> DS (pin 14)
#define SHIFT_DATA_Port     RGB_RED_GPIO_Port
#define SHIFT_CLOCK_Pin     RGB_GREEN_Pin     // PE11 -> SHCP (pin 11)
#define SHIFT_CLOCK_Port    RGB_GREEN_GPIO_Port
#define SHIFT_LATCH_Pin     RGB_BLUE_Pin      // PE13 -> STCP (pin 12)
#define SHIFT_LATCH_Port    RGB_BLUE_GPIO_Port

#define MSB 7
#define LSB 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
volatile uint32_t delayTime = 20;
volatile uint8_t pwm_counter = 0;
// PWM duty cycle per ogni colore (0-100)
volatile uint8_t duty_red = 0;
volatile uint8_t duty_green = 0;
volatile uint8_t duty_blue = 0;
// Flag per suonare melodia (settato da interrupt)
volatile uint8_t playMelodyFlag = 0;
// Contatore per alternare melodie (0 = Imperial March, 1 = AC/DC)
volatile uint8_t melodySelector = 0;

volatile int delayLed = 100;
// Handle per il task buzzer
osThreadId buzzerTaskHandle;
osThreadId breathTaskHandle;
osThreadId knightRiderTaskHandle;

// UART Interrupt - ricezione comandi
volatile uint8_t rxByte;           // Byte ricevuto
volatile uint8_t rxBuffer[32];     // Buffer comando
volatile uint8_t rxIndex = 0;      // Indice nel buffer
volatile uint8_t cmdReady = 0;     // Flag: comando pronto
volatile uint8_t manualColorMode = 1; // 1 = colore manuale, 0 = ciclo automatico
volatile uint8_t manualBreathMode = 0; // 1 = breath mode, 0 = no breath

osMutexId shiftMutex; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C3_Init(void);
static void MX_LTDC_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM7_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void delay_us(uint32_t us);
void playTone(uint16_t frequency, uint16_t duration_ms);
void playImperialMarch(void);
void playLongWayToTheTop(void);
void playBachToccata(void);
void BuzzerTask(void const * argument);
void shiftOut(uint8_t data);
void breath(void const * argument);
void KnightRiderEffect(void const * argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Retargeting printf su UART
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}

// Delay in microsecondi usando DWT cycle counter
void delay_us(uint32_t us)
{
  uint32_t startTick = DWT->CYCCNT;
  uint32_t delayTicks = us * (SystemCoreClock / 1000000);
  while ((DWT->CYCCNT - startTick) < delayTicks);
}

// Suona una nota alla frequenza specificata per la durata indicata
void playTone(uint16_t frequency, uint16_t duration_ms)
{
  if (frequency == 0) {
    // Pausa: nessun suono
    osDelay(duration_ms);
    return;
  }

  uint32_t period_us = 1000000 / frequency;
  uint32_t half_period_us = period_us / 2;
  uint32_t cycles = (uint32_t)frequency * duration_ms / 1000;

  for (uint32_t i = 0; i < cycles; i++) {
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    delay_us(half_period_us);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    delay_us(half_period_us);
  }
}

// Suona la Marcia Imperiale (Star Wars)
void playImperialMarch(void)
{
  uint16_t note[] = {
    // Intro
    NOTE_G4, NOTE_G4, NOTE_G4,
    NOTE_Eb4, NOTE_Bb4, NOTE_G4, NOTE_Eb4, NOTE_Bb4, NOTE_G4,
    // Parte 1
    NOTE_D5, NOTE_D5, NOTE_D5,
    NOTE_Eb5, NOTE_Bb4, NOTE_G4, NOTE_Eb4, NOTE_Bb4, NOTE_G4,
    // Parte 2
    NOTE_G4, NOTE_G4, NOTE_G4, NOTE_G4,
    NOTE_Eb4, NOTE_Eb4, NOTE_G4, NOTE_Eb4, NOTE_G4,
    NOTE_REST,
    NOTE_A4, NOTE_A4, NOTE_A4, NOTE_A4,
    NOTE_Ab4, NOTE_Ab4, NOTE_G4, NOTE_G4,
    // Finale
    NOTE_G4, NOTE_Eb4, NOTE_Bb4, NOTE_G4
  };

  uint16_t durate[] = {
    500, 500, 500,
    350, 150, 500, 350, 150, 1000,
    500, 500, 500,
    350, 150, 500, 350, 150, 1000,
    250, 250, 250, 250,
    350, 150, 250, 250, 500,
    250,
    250, 250, 250, 250,
    350, 150, 500, 500,
    500, 350, 150, 1000
  };

  int numNote = sizeof(note) / sizeof(note[0]);
  for (int i = 0; i < numNote; i++) {
    if (playMelodyFlag) return;  // Interrompi se pulsante premuto
    playTone(note[i], durate[i]);
    osDelay(50);
  }
}

// Suona "It's a Long Way to the Top" (AC/DC) - Riff principale
void playLongWayToTheTop(void)
{
  // Riff rock in A - stile AC/DC
  uint16_t note[] = {
    // Riff 1 - "It's a long way..."
    NOTE_A4, NOTE_A4, NOTE_G4, NOTE_A4, NOTE_REST,
    NOTE_A4, NOTE_A4, NOTE_G4, NOTE_A4, NOTE_REST,
    NOTE_A4, NOTE_C5, NOTE_D5, NOTE_C5, NOTE_A4,
    NOTE_REST,

    // Riff 2 - Risposta
    NOTE_E4, NOTE_E4, NOTE_G4, NOTE_A4, NOTE_REST,
    NOTE_A4, NOTE_G4, NOTE_E4, NOTE_D4, NOTE_E4,
    NOTE_REST,

    // Riff 3 - Power chords style
    NOTE_A3, NOTE_A3, NOTE_A4, NOTE_A3,
    NOTE_G4, NOTE_G4, NOTE_A4, NOTE_G4,
    NOTE_A3, NOTE_A3, NOTE_B3, NOTE_C4, NOTE_D4,
    NOTE_REST,

    // Finale rock!
    NOTE_A4, NOTE_G4, NOTE_A4,
    NOTE_D5, NOTE_C5, NOTE_A4,
    NOTE_A4, NOTE_A4, NOTE_A4
  };

  uint16_t durate[] = {
    // Riff 1
    200, 200, 200, 400, 100,
    200, 200, 200, 400, 100,
    200, 200, 200, 200, 400,
    200,

    // Riff 2
    200, 200, 200, 400, 100,
    200, 200, 200, 200, 400,
    200,

    // Riff 3
    150, 150, 300, 150,
    150, 150, 300, 150,
    150, 150, 150, 150, 400,
    200,

    // Finale
    200, 200, 400,
    200, 200, 400,
    150, 150, 600
  };

  int numNote = sizeof(note) / sizeof(note[0]);
  for (int i = 0; i < numNote; i++) {
    if (playMelodyFlag) return;  // Interrompi se pulsante premuto
    playTone(note[i], durate[i]);
    osDelay(30);  // pausa corta per ritmo rock
  }
}

// Suona Toccata e Fuga in Re minore (J.S. Bach) - Intro famoso
void playBachToccata(void)
{
  // L'iconico intro: A-G-A poi discesa drammatica
  uint16_t note[] = {
    // "Da-da-DAAAA" - Il trillo iniziale
    NOTE_A4, NOTE_G4, NOTE_A4,
    NOTE_REST,
    NOTE_G4, NOTE_F4, NOTE_E4, NOTE_D4, NOTE_Cs4, NOTE_D4,
    NOTE_REST,

    // Seconda frase - più alta
    NOTE_A4, NOTE_G4, NOTE_A4,
    NOTE_REST,
    NOTE_E5, NOTE_D5, NOTE_Cs5, NOTE_D5,
    NOTE_REST,
    NOTE_A4, NOTE_G4, NOTE_F4, NOTE_E4, NOTE_D4,
    NOTE_REST,

    // Accordi drammatici discendenti
    NOTE_D4, NOTE_A3, NOTE_D3,
    NOTE_REST,
    NOTE_D4, NOTE_E4, NOTE_F4,
    NOTE_REST,
    NOTE_Bb4, NOTE_A4, NOTE_Gs4, NOTE_A4,
    NOTE_REST,

    // Finale drammatico
    NOTE_E4, NOTE_F4, NOTE_Cs4, NOTE_D4,
    NOTE_REST,
    NOTE_A3, NOTE_D3,
    NOTE_D3
  };

  uint16_t durate[] = {
    // Trillo iniziale
    100, 100, 800,
    200,
    120, 120, 120, 120, 120, 600,
    300,

    // Seconda frase
    100, 100, 800,
    200,
    150, 150, 150, 600,
    200,
    120, 120, 120, 120, 500,
    300,

    // Accordi drammatici
    300, 300, 600,
    200,
    200, 200, 400,
    200,
    150, 150, 150, 600,
    300,

    // Finale
    200, 200, 200, 600,
    200,
    400, 400,
    1000
  };

  int numNote = sizeof(note) / sizeof(note[0]);
  for (int i = 0; i < numNote; i++) {
    if (playMelodyFlag) return;  // Interrompi se pulsante premuto
    playTone(note[i], durate[i]);
    osDelay(20);
  }
}


void KnightRiderEffect(void const * argument)
{
  printf("KnightRiderEffect Task avviato!\r\n");
  for(;;)
  {
    // === CHENILLARD (Knight Rider) - solo in modalita' auto ===
    if (manualColorMode)
    {
      shiftOut(0x00);
      
      // Andata: LED 0 -> 7
      for (int i = 0; i < 8 && !manualBreathMode; i++)
      {
        osDelay(delayLed);
        shiftOut(1 << i);
        // Controlla comandi durante animazione
      }
      // Ritorno: LED 6 -> 0
      for (int i = 7; i >= 0 && !manualBreathMode; i--)
      {
        osDelay(delayLed);
        shiftOut(1 << i);
        // Controlla comandi durante animazione
      }
    }
    osDelay(50);
  }
}


void breath(void const * argument)
{
  printf("BreathTask avviato!\r\n");
  for(;;)
  {
    if(manualBreathMode)
    {
      shiftOut(0x00);
      // === ACCENSIONE: LED si accendono uno dopo l'altro ===
      // 0b00000001 -> 0b00000011 -> 0b00000111 -> ... -> 0b11111111
      for (int i = 0; i < 8 && !manualColorMode; i++)
      {
        osDelay(delayLed);
        shiftOut((1 << (i + 1)) - 1);  // Accende LED da 0 a i
        
      }

      // === SPEGNIMENTO: LED si spengono dall'ultimo al primo ===
      // 0b11111111 -> 0b01111111 -> 0b00111111 -> ... -> 0b00000000
      for (int i = 7; i >= 0 && !manualColorMode; i--)
      {
        osDelay(delayLed);
        shiftOut((1 << i) - 1);  // Spegne LED da 7 fino a i
        
      }
    }
    osDelay(50);
  }
}

// Invia un byte al 74HC595 (bit-banging)
void shiftOut(uint8_t data)
{
  osMutexWait(shiftMutex, osWaitForever);
  // Assicura che CLOCK e LATCH siano LOW all'inizio
  HAL_GPIO_WritePin(SHIFT_CLOCK_Port, SHIFT_CLOCK_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SHIFT_LATCH_Port, SHIFT_LATCH_Pin, GPIO_PIN_RESET);

  // Manda 8 bit, MSB first (Q7=MSB, Q0=LSB)
  for (int i = MSB; i >= LSB; i--)
  {
    // Setta DATA in base al bit corrente
    if (data & (1 << i))
      HAL_GPIO_WritePin(SHIFT_DATA_Port, SHIFT_DATA_Pin, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(SHIFT_DATA_Port, SHIFT_DATA_Pin, GPIO_PIN_RESET);

    // Piccolo delay per stabilizzare DATA
    delay_us(1);

    // Impulso di clock: LOW -> HIGH (dato entra) -> LOW
    HAL_GPIO_WritePin(SHIFT_CLOCK_Port, SHIFT_CLOCK_Pin, GPIO_PIN_SET);
    delay_us(1);
    HAL_GPIO_WritePin(SHIFT_CLOCK_Port, SHIFT_CLOCK_Pin, GPIO_PIN_RESET);
  }

  // Impulso di latch per trasferire alle uscite
  delay_us(1);
  HAL_GPIO_WritePin(SHIFT_LATCH_Port, SHIFT_LATCH_Pin, GPIO_PIN_SET);
  delay_us(1);
  HAL_GPIO_WritePin(SHIFT_LATCH_Port, SHIFT_LATCH_Pin, GPIO_PIN_RESET);
  osMutexRelease(shiftMutex);
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
  MX_CRC_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  // Abilita DWT cycle counter per delay_us preciso
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // Test printf retargeting
  printf("Ciao dal STM32! (via printf)\r\n");

  // Avvia ricezione UART con interrupt (1 byte alla volta)
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxByte, 1);

  printf("Comandi: R G B W O (colore), A (auto), P (play)\r\n");
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 4096);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  // Task per il buzzer (gira in parallelo al task RGB)
  osThreadDef(buzzerTask, BuzzerTask, osPriorityNormal, 0, 512);
  buzzerTaskHandle = osThreadCreate(osThread(buzzerTask), NULL);


  osThreadDef(breathTask, breath, osPriorityNormal, 0, 512);
  breathTaskHandle = osThreadCreate(osThread(breathTask), NULL);

  osThreadDef(knightRiderTask, KnightRiderEffect, osPriorityNormal, 0, 512);
  knightRiderTaskHandle = osThreadCreate(osThread(knightRiderTask), NULL);
  
  osMutexDef(shiftMutex);                                                                                                                                                                                                                     
  shiftMutex = osMutexCreate(osMutex(shiftMutex)); 
  
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);  // <- AGGIUNGI QUESTA
    //HAL_Delay(500);    
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 9;
  hltdc.Init.VerticalSync = 1;
  hltdc.Init.AccumulatedHBP = 29;
  hltdc.Init.AccumulatedVBP = 3;
  hltdc.Init.AccumulatedActiveW = 269;
  hltdc.Init.AccumulatedActiveH = 323;
  hltdc.Init.TotalWidth = 279;
  hltdc.Init.TotalHeigh = 327;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 240;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 320;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.FBStartAdress = 0xD0000000;
  pLayerCfg.ImageWidth = 240;
  pLayerCfg.ImageHeight = 320;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 71;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 99;
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
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, BUZZER_Pin|RGB_RED_Pin|RGB_GREEN_Pin|RGB_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACP_RST_GPIO_Port, ACP_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RDX_Pin|WRX_DCX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUZZER_Pin RGB_RED_Pin RGB_GREEN_Pin RGB_BLUE_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|RGB_RED_Pin|RGB_GREEN_Pin|RGB_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin CSX_Pin OTG_FS_PSO_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MEMS_INT1_Pin MEMS_INT2_Pin TP_INT1_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin|MEMS_INT2_Pin|TP_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ACP_RST_Pin */
  GPIO_InitStruct.Pin = ACP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACP_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OC_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TE_Pin */
  GPIO_InitStruct.Pin = TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RDX_Pin WRX_DCX_Pin */
  GPIO_InitStruct.Pin = RDX_Pin|WRX_DCX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD4_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == B1_Pin)
  {
    // Solo setta il flag! Niente funzioni bloccanti in ISR
    playMelodyFlag = 1;
  }
}

// Callback UART - chiamata quando arriva un byte
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    // Comando completo? (termina con Enter)
    if (rxByte == '\r' || rxByte == '\n')
    {
      if (rxIndex > 0)
      {
        rxBuffer[rxIndex] = '\0';  // Termina stringa
        cmdReady = 1;              // Segnala comando pronto
        rxIndex = 0;
      }
    }
    else
    {
      // Accumula caratteri nel buffer
      if (rxIndex < sizeof(rxBuffer) - 1)
      {
        rxBuffer[rxIndex++] = rxByte;
      }
    }

    // Riavvia ricezione per il prossimo byte
    HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxByte, 1);
  }
}

// Task dedicato al buzzer - gira in parallelo al task RGB
void BuzzerTask(void const * argument)
{
  printf("BuzzerTask avviato!\r\n");

  for(;;)
  {
    // Aspetta che il flag venga settato (polling ogni 50ms)
    if (playMelodyFlag)
    {
      playMelodyFlag = 0;  // Reset flag

      switch (melodySelector)
      {
        case 0:
          printf("Imperial March!\r\n");
          playImperialMarch();
          printf("The Dark Side is strong!\r\n");
          break;

        case 1:
          printf("Long Way to the Top - AC/DC!\r\n");
          playLongWayToTheTop();
          printf("Rock'n'Roll!\r\n");
          break;

        case 2:
          printf("Toccata e Fuga - J.S. Bach!\r\n");
          playBachToccata();
          printf("Barocco puro!\r\n");
          break;
      }

      // Alterna per la prossima volta (3 melodie)
      melodySelector = (melodySelector + 1) % 3;
    }

    osDelay(50);  // Controlla ogni 50ms
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
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 5 */

  printf("74HC595 Shift Register - Chenillard 8 LED\r\n");
  printf("Comandi: 0-9 (pattern), A (auto), P (play)\r\n");

  // Inizializza: tutte le LED spente
  shiftOut(0x00);
  for(;;)
  {
    // === CONTROLLO COMANDI UART ===
    if (cmdReady)
    {
      cmdReady = 0;
      printf("Ricevuto: %s\r\n", rxBuffer);

      if (rxBuffer[0] == 'P' || rxBuffer[0] == 'p')
      {
        playMelodyFlag = 1;
        printf("PLAY melodia!\r\n");
      }
      else if (rxBuffer[0] == 'A' || rxBuffer[0] == 'a')
      {
        manualColorMode = 1;
        manualBreathMode = 0;
        printf("Chenillard AUTO\r\n");
      }
      else if(rxBuffer[0] == '+')
      {
        // Diminuire velocita'
        if (delayLed < 1000)
          delayLed += 50;
        else
          delayLed = 1000;
        printf("Delay LED: %d ms\r\n", delayLed);

      }
      else if(rxBuffer[0] == '-')
      {
        // Diminuire velocita'
        if (delayLed > 50)
          delayLed -= 50;
        else delayLed = 50;
        printf("Delay LED: %d ms\r\n", delayLed);

      }
      else if(rxBuffer[0] == 'B' || rxBuffer[0] == 'b')
      {
        manualBreathMode = 1;
        manualColorMode = 0;  // Disattiva pattern manuale
        printf("Breath mode ON\r\n");
      }
      else if (rxBuffer[0] >= '0' && rxBuffer[0] <= '9')
      {
        // Pattern manuali
        uint8_t patterns[] = {
          0x00,  // 0: tutto spento
          0xFF,  // 1: tutto acceso
          0xAA,  // 2: alternato 10101010
          0x55,  // 3: alternato 01010101
          0x0F,  // 4: meta' bassa
          0xF0,  // 5: meta' alta
          0x18,  // 6: centro
          0x81,  // 7: estremi
          0x3C,  // 8: centro largo
          0xC3   // 9: estremi larghi
        };
        uint8_t pattern = patterns[rxBuffer[0] - '0'];
        manualColorMode = 1;  // Blocca chenillard
        manualBreathMode = 0; // Disattiva breath
        printf("Pattern %c: 0x%02X (manuale)\r\n", rxBuffer[0], pattern);
        shiftOut(pattern);
      }
    }

      osDelay(50);  // In modalita' manuale, aspetta comandi
    
  }             


  /* Infinite loop */
  // for(;;)
  // {
  //   HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
  //   HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
  //   osDelay(delayTime);  // <- USA LA VARIABILE invece di 500 fisso
    
  //   HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
  //   HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
  //   osDelay(delayTime);  // <- USA LA VARIABILE invece di 500 fisso
  // }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM7)
  {
    pwm_counter++;
    if (pwm_counter >= 100) pwm_counter = 0;

    // PWM per LED RGB
    HAL_GPIO_WritePin(RGB_RED_GPIO_Port, RGB_RED_Pin,
                      (pwm_counter < duty_red) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RGB_GREEN_GPIO_Port, RGB_GREEN_Pin,
                      (pwm_counter < duty_green) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RGB_BLUE_GPIO_Port, RGB_BLUE_Pin,
                      (pwm_counter < duty_blue) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  /* USER CODE END Callback 1 */
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
#ifdef USE_FULL_ASSERT
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
