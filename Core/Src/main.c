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
volatile uint8_t uTIM1_Interrupt_Flag = 0; /* Flag for TIM1 10us Interrupt */
volatile uint16_t uADC_Value[2] = {0}; /* Changed to 16-bit to match DMA HalfWord alignment */
volatile uint32_t uADC_T2P_Average = 0; 
volatile uint32_t uADC_PWGD_Average = 0;
volatile uint32_t uADC_Sum_T2P = 0;
volatile uint32_t uADC_Sum_PWGD = 0;
volatile uint32_t uADC_Value_T2P = 0;
volatile uint32_t uADC_Value_PWGD = 0;
volatile uint16_t uADC_Count = 0;
volatile uint32_t uADC_Processing_Value = 0; /* New variable for user processing */
volatile uint32_t uADC_Sum = 0; /* Accumulator for averaging */
volatile uint32_t uADC_Average = 0; /* Final averaged value */
volatile uint32_t uLED_Count = 0; /* Counter for LED timing */
volatile uint8_t Powerkeyin = 0;
volatile uint8_t Powerkeyinstate = 0;
volatile uint8_t Powerkeyinstate_prev = 0;
volatile uint8_t ap_seq_state = 0;
volatile uint32_t ap_seq_timer = 0;
volatile uint32_t ap_target_delay = 0;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_usart1_tx;
uint8_t usart_tx_buffer[16]; /* Increased buffer size */
/* IWDG_HandleTypeDef hiwdg; */ /* Removed HAL handle to use Registers directly */

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
void LED_Control(uint8_t on);
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
  /* Debug: Init LED (PB0) immediately */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /* Check functionality by blinking using the requested method */
  /* ON: Output OD Low */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, led_Pin, GPIO_PIN_RESET); /* ON */
  HAL_Delay(100); 
  
  /* OFF: Input High-Z */
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  
  /* Initial State: PA6 High, PA7 High */
  RELAY_CLOSE();
  DCDC_DISABLE();
  LB16F1_LED_OFF();
  AP_OFF(); 

  /* Enable ADC DMA for 2 Channels */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)uADC_Value, 2);
  /* Disable DMA Interrupts to prevent CPU overload (92kHz IRQ flood) */
  __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);
  /* Disable ADC Interrupts */
  __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_OVR | ADC_IT_EOC | ADC_IT_EOS);

  /* HAL_UART_Transmit(&huart1, (uint8_t*)"ADC Init Skip\n", 14, 100); */

  HAL_TIM_Base_Start_IT(&htim1);
  /* HAL_UART_Transmit(&huart1, (uint8_t*)"TIM Start\n", 10, 100); */

  /* State Machine Variables */
  uint8_t state_main = 1;
  //uint32_t state_timer = 0;
  uint32_t led_timer = 0;
  uint32_t state4_timer = 0;
  uint32_t pwgd_debounce = 0;
  uint32_t pwgd_fault_timer = 0; /* Timer for State 3 fault detection */

  /* Force Enable Interrupts */
  /* Reconfigure Priorities to prevent starvation */
  HAL_NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 0, 0); /* Highest: Timing Critical */
  HAL_NVIC_SetPriority(ADC1_IRQn, 1, 0);                /* High: Data Acq */
  HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);              /* Lower: Comms */
  
  __enable_irq();

  /* Debug: Confirm Loop Entry */
  HAL_UART_Transmit(&huart1, (uint8_t*)"Loop Start\n", 11, 100);

  /* Startup LED Blink */
  for(int i=0; i<3; i++) {
      LED_Control(1); 
      HAL_Delay(100);
      LED_Control(0); 
      HAL_Delay(100);
  }
  /* Ensure LED is in correct initial state (OFF) */
  LED_Control(0);


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* Infinite loop */
  while (1)
  {      
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */

    /* State Machine Logic */
    if (uTIM1_Interrupt_Flag == 1)
    {
      DEBUG_PIN_HIGH();
      IWDG->KR = 0xAAAA; /* Feed Dog */
      uTIM1_Interrupt_Flag = 0;

      /*Input Sampling*/
      Powerkeyin = HAL_GPIO_ReadPin(Powerkeyin_GPIO_Port, Powerkeyin_Pin);
      uADC_Value_T2P = uADC_Value[0]&0x0FFF;
      uADC_Value_PWGD = uADC_Value[1]&0x0FFF;
      //HAL_UART_Transmit(&huart1, (uint8_t*)"int\n", 4, 100); 

/*AP power on off control*/
      /* Debounce Logic: 500ms @ 10us tick = 50000 ticks */
      static uint32_t pk_timer_low = 0;
      static uint32_t pk_timer_high = 0;

      if (Powerkeyin == 0)
      {
          pk_timer_high = 0;
          pk_timer_low++;
          if (pk_timer_low >= 50000)
          {
              Powerkeyinstate = 1; /* ON */
              pk_timer_low = 50000;
          }
      }
      else
      {
          pk_timer_low = 0;
          pk_timer_high++;
          if (pk_timer_high >= 50000)
          {
              Powerkeyinstate = 0; /* OFF */
              pk_timer_high = 50000;
          }
      }
      /*LB16F1 Key led control*/
      if (Powerkeyinstate == 1)
      {
          LB16F1_LED_ON();
      }
      else
      {
          LB16F1_LED_OFF();
      }

      /* AP Sequence Logic */
      /* Edge Detection */
      if (Powerkeyinstate != Powerkeyinstate_prev)
      {
          if (Powerkeyinstate == 1) /* 0 -> 1 */
          {
               ap_target_delay = 30000; /* 300ms */
          }
          else /* 1 -> 0 */
          {
               ap_target_delay = 800000; /* 8000ms */
          }
          Powerkeyinstate_prev = Powerkeyinstate;
          ap_seq_state = 1; /* Start Sequence */
          ap_seq_timer = 0;
      }

      /* State Machine */
      switch (ap_seq_state)
      {
          case 0: /* IDLE */
              break;

          case 1: /* OFF 10ms */
              AP_OFF();
              ap_seq_timer++;
              if (ap_seq_timer >= 1000) /* 10ms */
              {
                  ap_seq_timer = 0;
                  ap_seq_state = 2;
              }
              break;

          case 2: /* ON Delay */
              AP_ON();
              ap_seq_timer++;
              if (ap_seq_timer >= ap_target_delay)
              {
                  ap_seq_timer = 0;
                  ap_seq_state = 3;
              }
              break;

          case 3: /* OFF 10ms */
              AP_OFF();
              ap_seq_timer++;
              if (ap_seq_timer >= 1000) /* 10ms */
              {
                  ap_seq_timer = 0;
                  ap_seq_state = 0; /* Back to IDLE */
              }
              break;
      }

      /*UART send for debug*/
      #ifdef MY_DE_BUG
      static uint32_t usart_timer = 0;
      usart_timer++;
      if(usart_timer > 100000)
      {
    	  usart_timer = 0;
    	  HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&uADC_T2P_Average, 4);
      }
      #endif

      /*POE power classification and relay DCDC enable control*/
      switch (state_main)
      {
        case 1: /* Wait for PWGD > 1.5V */
          /* LED: 1s Blink (500ms ON / 500ms OFF) - 100kHz * 0.5s = 50000 ticks */
          led_timer++;
          if (led_timer < 50000) LED_Control(1); /* ON */
          else if (led_timer < 100000) LED_Control(0); /* OFF */
          else led_timer = 0;

          /* PWGD Check: > 1.5V (~1861 @ 3.3V) */
          if (uADC_Value_PWGD > 0x7ff) 
          {
             pwgd_debounce++;
             //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);//dcdc en
             //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); //relay close
             RELAY_CLOSE();
             DCDC_ENABLE();
          } 
          else 
          {
             if(pwgd_debounce > 1000) pwgd_debounce -= 1000; // 0.01s;3ms resume
             //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);//dcdc dis
             //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); //relay open
             //RELAY_OPEN();
             DCDC_DISABLE();
          }

          /* 3s debounce = 3s / 10us tick = 300000 ticks */
          if (pwgd_debounce >= 310000) 
          {
            pwgd_debounce = 0;
            state_main = 2;
            led_timer = 0; /* Reset for next state */
            HAL_UART_Transmit_DMA(&huart1, (uint8_t*)"S2\n", 3);
          }
          break;

        case 2: /* Check T2P Power Level */
          /* LED: 2s Blink (1s ON / 1s OFF) -> 100000 ticks */
         // led_timer++;
          //if (led_timer < 100000) LED_Control(1);
          //else if (led_timer < 200000) LED_Control(0);
          //else led_timer = 0;
          
          /* Check T2P (uADC_T2P_Average) */
          //uADC_Sum_T2P += uADC_Value_T2P;
          if(uADC_Value_T2P > 0x7ff) uADC_Sum_T2P += 0xfff;
          else uADC_Sum_T2P += 0;
          uADC_Count++;
          /* 8192 samples * 10us = 81.92ms */
          if(uADC_Count >= 8192)
          {
            uADC_T2P_Average = uADC_Sum_T2P >> 13;
            uADC_Count = 0;
            uADC_Sum_T2P = 0;

            if (uADC_T2P_Average > 2866 && uADC_T2P_Average < 3276) /*70%~80%*/
            //if (uADC_T2P_Average < 200) /*20%~30%*/
            {
               state_main = 3;
               led_timer = 0;
               pwgd_fault_timer = 0; /* Reset fault timer on entry */
               HAL_UART_Transmit_DMA(&huart1, (uint8_t*)"71W->S3\n", 8);
              
            }
            else
            {
               state_main = 4;
               led_timer = 0;
               HAL_UART_Transmit_DMA(&huart1, (uint8_t*)"Other->S4\n", 10);
            }
          }
          break;

        case 3: /* 71W Mode */
          /* LED: 3s Blink (1.5s ON / 1.5s OFF) - 1.5s / 10us = 150000 ticks */
          led_timer++;
          if (led_timer < 300000) LED_Control(1);
          else if (led_timer < 600000) LED_Control(0);
          else led_timer = 0;
          
          /* PWGD Fault Monitor: Reset if < 1.6V for 3s */
          if (uADC_Value_PWGD < 0x7ff) 
          {
              pwgd_fault_timer++;
              if (pwgd_fault_timer >= 300000) /* 3s * 100kHz */
              {
                  pwgd_fault_timer = 0;
                  state_main = 1; /* Reset */
                  //HAL_UART_Transmit_DMA(&huart1, (uint8_t*)"PWGD Fail->S1\n", 14);
              }
          }
          else
          {
              pwgd_fault_timer = 0;
          }

          //mcu will not power off,case 3 should return to case 1 after 10s//
          /* PA6 High, PA7 High */
          /* PA6 High, PA7 High */
          RELAY_CLOSE();
          DCDC_ENABLE(); //relay close and dcdc en
          break;

        case 4: /* Low Power / Fail Mode */
          /* LED Off */
          state4_timer++;
          if(state4_timer < 300000) LED_Control(0);  // delay 3s to renegotiate with pse
          else
          {
        	  state4_timer = 0;
        	  state_main = 1;
          }
          /* PA6 Low, PA7 Low */
          /* PA6 Low, PA7 Low */
          RELAY_OPEN();
          DCDC_DISABLE(); //relay open and dcdc off
          break;
      }
    }
    /* */
    IWDG->KR = 0xAAAA; /* Feed Dog */
    DEBUG_PIN_LOW();
  }
  /* USER CODE END 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

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

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_39CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_39CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analog WatchDog 1
  */
  AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.Channel = ADC_CHANNEL_0;
  AnalogWDGConfig.ITMode = DISABLE;
  AnalogWDGConfig.HighThreshold = 0;
  AnalogWDGConfig.LowThreshold = 0;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63; /* 64MHz / 64 = 1MHz */
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9;      /* 1MHz / 10 = 100kHz (10us) */
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
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
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
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{
  /* IWDG Initialization using Registers (No HAL driver needed) */
  /* 1. Enable IWDG (LSI automatically enabled by hardware if not already) */
  IWDG->KR = 0xCCCC; /* Start IWDG */
  
  /* 2. Enable register access */
  IWDG->KR = 0x5555; 
  
  /* 3. Set Prescaler (32kHz / 32 = 1kHz) */
  /* PR: 000->/4, 011->/32 */
  IWDG->PR = 0x03; 
  
  /* 4. Set Reload (1000ms = 1000 counts at 1kHz) */
  IWDG->RLR = 1000; 

  /* 5. Reload Counter */
  IWDG->KR = 0xAAAA;
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET); /* Default OFF (High-Z) */

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 (Powerkeyin) */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  #ifdef MY_DE_BUG
  /*Configure GPIO pin : PA12 (Debug Measurement) */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  #endif

  /*Configure GPIO pin : led_Pin */

  /*Configure GPIO pin : led_Pin */
  GPIO_InitStruct.Pin = led_Pin; 
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);



  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void LED_Control(uint8_t on)
{
    static uint8_t current_state = 0xFF; /* 0xFF = Unknown/Init */

    if (current_state == on) return; /* No change needed */

    current_state = on;

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = led_Pin;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    if(on)
    {
        /* ON: Output Open-Drain, Low */
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        HAL_GPIO_WritePin(GPIOB, led_Pin, GPIO_PIN_RESET);
    }
    else
    {
        /* OFF: Input, High-Z */
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    uTIM1_Interrupt_Flag = 1;

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

  /* Force Init LED PB0 in case we crashed before MX_GPIO_Init */
  /* LED_Control handles Init, enabling Clock if needed is good practice though LED_Control relies on it being enabled? 
     LED_Control does NOT enable clock. We should enable it here. */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  while (1)
  {
      /* Blink LED fast to indicate Error Loop */
      LED_Control(1); /* ON */
      for(volatile int i=0; i<100000; i++); 
      LED_Control(0); /* OFF */
      for(volatile int i=0; i<100000; i++); 
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
