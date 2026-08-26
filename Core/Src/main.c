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
#include "vcnl4040.h"
#include "shoot_detect.h"
#include "can_protocol.h"
#include "led_rgb.h"
#include "ws2812_uart.h"
#include "reliability.h"
#include <string.h>
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
CAN_HandleTypeDef hcan;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;
IWDG_HandleTypeDef hiwdg;

/* USER CODE BEGIN PV */
/* Global detection instance — accessed by ISR callbacks */
ShootDetect_t g_shoot_detect;
ShootData_Report_t g_shoot_report;
CAN_ErrorStats_t g_can_stats = {0};

/* 10 Hz status-report tick (set by TIM14 ISR, cleared by main loop) */
volatile bool g_sensor_tick_10hz = false;

/* 10 Hz LED refresh tick (set by TIM15 ISR, cleared by main loop) */
volatile bool g_led_tick_10hz = false;

/* CAN ready flag: false when CAN bus is not connected */
volatile bool g_can_ready = false;

/* Threshold debug switch. Default on in this firmware build.
   It suppresses automatic strong-fault resets and does not transmit 0x200. */
volatile bool debug_flag = true;

/* Live sensor data (watch in debugger at runtime) */
volatile uint16_t g_dbg_front_prox = 0;
volatile uint16_t g_dbg_rear_prox  = 0;
/* Peak valid PS_DATA values since this MCU boot; inspect in the debugger. */
volatile uint16_t g_dbg_front_prox_max = 0;
volatile uint16_t g_dbg_rear_prox_max  = 0;
/* Initial 0xFFFF means no valid sample has been seen since this boot. */
volatile uint16_t g_dbg_front_prox_min = 0xFFFFU;
volatile uint16_t g_dbg_rear_prox_min  = 0xFFFFU;
/* Incremented once per full front+rear I2C sample pair. */
volatile uint32_t g_dbg_sensor_sample_count = 0;
/* Baselines captured by ShootDetect_Calibrate at this MCU boot. */
volatile uint16_t g_dbg_front_baseline = 0;
volatile uint16_t g_dbg_rear_baseline  = 0;
volatile uint16_t g_dbg_front_threshold = 0;
volatile uint16_t g_dbg_rear_threshold  = 0;
/* Debug watch variable: valid shots counted since the latest power-up/reset. */
volatile uint32_t gbd_shoot_count = 0;

#define DEBUG_SENSOR_FLASH_MS 200U

/* VOFA+ JustFloat over USART2: front PS_DATA, rear PS_DATA, frame tail. */
#define VOFA_UART_BAUDRATE    1000000U
#define VOFA_CHANNEL_COUNT    8U
#define VOFA_FRAME_SIZE       (VOFA_CHANNEL_COUNT * 4U + 4U)

/* Hardware-noise isolation build: after sensor initialization, run only the
   two raw PS_DATA reads and the existing VOFA+ stream.  This deliberately
   skips calibration, CAN, timers, LED activity, watchdog, adaptive
   thresholds, shot detection, and fault handling. */
#define DEBUG_SENSOR_UART_ONLY 1

/* Debug LED override — write from debugger:
   g_led_cmd = 1 → 9 LEDs all green  (WS2812_COLOR(255,0,0))
            = 2 → 9 LEDs all red    (WS2812_COLOR(0,255,0))
            = 3 → 9 LEDs all blue   (WS2812_COLOR(0,0,255))
            = 4 → team color (current team)
            = 5 → all off
            = 0 → normal operation (自动)
   Set g_led_cmd_count to limit lit LEDs (0 = all 9). */
volatile uint8_t  g_led_cmd        = 0;
volatile uint8_t  g_led_cmd_count  = 0;
volatile uint8_t  g_heat_debug     = 0;   /* manual heat data (0–255)           */
/* USER CODE END PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

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

  /* Read the reset cause and any retained HardFault/CAN Bus-Off note. */
  Reliability_EarlyInit();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM14_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  /* ---- NVIC configuration ---- */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);   /* EXTI: PB5/PB12 (highest)   */
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  HAL_NVIC_SetPriority(TIM14_IRQn, 3, 0);      /* TIM14: 10 Hz sensor tick   */
  HAL_NVIC_EnableIRQ(TIM14_IRQn);
  HAL_NVIC_SetPriority(TIM15_IRQn, 3, 0);      /* TIM15: 10 Hz LED tick       */
  HAL_NVIC_EnableIRQ(TIM15_IRQn);
  HAL_NVIC_SetPriority(CEC_CAN_IRQn, 2, 0);    /* CAN: referee comms         */
  HAL_NVIC_EnableIRQ(CEC_CAN_IRQn);

  /* Start TIM16 free-running counter (PSC/ARR set by MX_TIM16_Init) */
  __HAL_TIM_ENABLE(&htim16);

  /* ---- Init VCNL4040 sensors ---- */
  bool front_sensor_ready = (VCNL4040_Init(&hi2c1) == HAL_OK);
  bool rear_sensor_ready  = (VCNL4040_Init(&hi2c2) == HAL_OK);
  Reliability_SetWeakFault(REL_WEAK_FRONT_SENSOR, !front_sensor_ready);
  Reliability_SetWeakFault(REL_WEAK_REAR_SENSOR, !rear_sensor_ready);

  /* ---- Init projectile detection ---- */
  /* front_i2c=hi2c1 (PB6/PB7, 2nd in path), rear_i2c=hi2c2 (PB13/PB14, 1st) */
  ShootDetect_Init(&g_shoot_detect, &hi2c1, &hi2c2);
  ShootDetect_SetParams(&g_shoot_detect,
                        DEFAULT_SPEED_MIN_MPS,
                        DEFAULT_SPEED_MAX_MPS,
                        DEFAULT_TIMEOUT_MS);

#if DEBUG_SENSOR_UART_ONLY
  /* WS2812 LEDs retain their last frame across MCU reset.  Clear them once
     before entering the sensor-only loop; no LED application or refresh task
     runs afterwards. */
  {
      uint32_t led_off[LED_COUNT] = {0};
      ws2812_uart_init();
      ws2812_uart_send(led_off, LED_COUNT);
      HAL_Delay(1);
  }

  /* Raw sensor isolation loop.  Keep the existing 8-channel JustFloat frame
     format so the current VOFA+ configuration remains usable:
       channel 0 = front PS_DATA, channel 1 = rear PS_DATA,
       channels 2..7 = zero (no calibration/derived values in this mode). */
  while (1)
  {
      uint8_t vofa_frame[VOFA_FRAME_SIZE];
      float vofa_values[VOFA_CHANNEL_COUNT] = {0.0f};
      uint32_t vofa_tail = 0x7F800000U;

      g_dbg_front_prox = VCNL4040_GetProximity(&hi2c1);
      g_dbg_rear_prox  = VCNL4040_GetProximity(&hi2c2);
      g_dbg_sensor_sample_count++;


      vofa_values[0] = (float)g_dbg_front_prox;
      vofa_values[1] = (float)g_dbg_rear_prox;
      memcpy(&vofa_frame[0], vofa_values, sizeof(vofa_values));
      memcpy(&vofa_frame[VOFA_CHANNEL_COUNT * 4U], &vofa_tail,
             sizeof(vofa_tail));
      (void)HAL_UART_Transmit(&huart2, vofa_frame, VOFA_FRAME_SIZE, 1U);
  }
#endif

  /* ---- Calibrate baseline & set thresholds ---- */
  if (front_sensor_ready && rear_sensor_ready &&
      !ShootDetect_Calibrate(&g_shoot_detect)) {
      Reliability_SetWeakFault(REL_WEAK_SHOOT_DETECT, true);
  }
  g_dbg_front_baseline = g_shoot_detect.front_baseline;
  g_dbg_rear_baseline  = g_shoot_detect.rear_baseline;

  /* ---- Init success: light IND_1 (PA5 DAC_OUT2, max = on) ---- */
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

  /* ---- Init LED strip (WS2812 via USART3 PB10) ---- */
  LedStrip_Init();
  LedStrip_SetTeam(TEAM_BLUE);

  /* ---- Init CAN protocol (slave-only) ---- */
  CANProtocol_Init(&hcan);
  /* Accept standard CAN data frames for the protocol RX interrupt. */
  CAN_FilterTypeDef can_filter = {0};
  can_filter.FilterBank = 0;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;
  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  can_filter.FilterActivation = ENABLE;
  can_filter.SlaveStartFilterBank = 0;
  if (HAL_CAN_ConfigFilter(&hcan, &can_filter) != HAL_OK) {
      Error_Handler();
  }
  /* Start CAN and enable receive interrupt. */
  if (HAL_CAN_Start(&hcan) != HAL_OK) {
      Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan,
        CAN_IT_RX_FIFO0_MSG_PENDING |
        CAN_IT_ERROR_WARNING |
        CAN_IT_ERROR_PASSIVE |
        CAN_IT_BUSOFF |
        CAN_IT_LAST_ERROR_CODE) != HAL_OK) {
      Error_Handler();
  }
  g_can_ready = true;
  g_can_stats.state = (uint8_t)HAL_CAN_GetState(&hcan);

  /* Threshold-debug builds do not start IWDG; IWDG cannot be disabled later. */
  if (!debug_flag) {
      MX_IWDG_Init();
  }

  /* Report a prior strong reset before announcing this successful startup. */
  uint8_t pending_strong = Reliability_GetPendingStrongMask();
  if (pending_strong != 0U) {
      (void)CANProtocol_SendStrongFault(pending_strong);
  }
  (void)CANProtocol_SendBoot();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_led_shot_count = 0U;
  uint32_t last_led_front_int_count = 0U;
  uint32_t last_led_rear_int_count = 0U;
  uint32_t debug_sensor_flash_until = 0U;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Always check timeout + clear sensor interrupts (fast) */
    ShootDetect_Process(&g_shoot_detect);

    /* A 0x220 request is deferred from the CAN ISR because calibration reads
       both sensors 20 times and must never block CAN interrupt handling. */
    if (CANProtocol_TakeCalibrationRequest()) {
        bool calibration_ok = ShootDetect_Calibrate(&g_shoot_detect);
        Reliability_SetWeakFault(REL_WEAK_SHOOT_DETECT, !calibration_ok);
        g_dbg_front_baseline  = g_shoot_detect.front_baseline;
        g_dbg_rear_baseline   = g_shoot_detect.rear_baseline;
        g_dbg_front_threshold = g_shoot_detect.front_threshold_low;
        g_dbg_rear_threshold  = g_shoot_detect.rear_threshold_low;
        (void)CANProtocol_SendCalibrationAck(calibration_ok ? 0x00U : 0x01U);
    }

    /* Fast threshold-debug sampling.  Each current/min/max update comes from
       the same I2C read, so extrema have exactly the sampling rate of PS_DATA. */
    g_dbg_front_prox = VCNL4040_GetProximity(&hi2c1);
    if (g_dbg_front_prox != 0xFFFFU) {
        if (g_dbg_front_prox > g_dbg_front_prox_max) {
            g_dbg_front_prox_max = g_dbg_front_prox;
        }
        if (g_dbg_front_prox < g_dbg_front_prox_min) {
            g_dbg_front_prox_min = g_dbg_front_prox;
        }
    }

    g_dbg_rear_prox = VCNL4040_GetProximity(&hi2c2);
    if (g_dbg_rear_prox != 0xFFFFU) {
        if (g_dbg_rear_prox > g_dbg_rear_prox_max) {
            g_dbg_rear_prox_max = g_dbg_rear_prox;
        }
        if (g_dbg_rear_prox < g_dbg_rear_prox_min) {
            g_dbg_rear_prox_min = g_dbg_rear_prox;
        }
    }
    g_dbg_sensor_sample_count++;

    ShootDetect_UpdateAdaptiveThresholds(&g_shoot_detect,
                                         g_dbg_front_prox,
                                         g_dbg_rear_prox,
                                         HAL_GetTick());
    g_dbg_front_baseline = g_shoot_detect.front_baseline;
    g_dbg_rear_baseline  = g_shoot_detect.rear_baseline;
    g_dbg_front_threshold = g_shoot_detect.front_threshold_low;
    g_dbg_rear_threshold  = g_shoot_detect.rear_threshold_low;

    /* CAN Bus-Off is shown locally for 2 s, then the MCU restarts. */
    if (!debug_flag && Reliability_ShouldResetNow()) {
        NVIC_SystemReset();
    }

    uint32_t now_tick = HAL_GetTick();

    /* Stream adaptive-threshold diagnostics as VOFA+ JustFloat channels:
       0 front PS, 1 rear PS, 2 front baseline, 3 rear baseline,
       4 front THDL, 5 rear THDL, 6 front PS-THDL, 7 rear PS-THDL. */
    if (debug_flag) {
        uint8_t vofa_frame[VOFA_FRAME_SIZE];
        float vofa_values[VOFA_CHANNEL_COUNT];
        uint32_t vofa_tail = 0x7F800000U;
        vofa_values[0] = (float)g_dbg_front_prox;
        vofa_values[1] = (float)g_dbg_rear_prox;
        vofa_values[2] = (float)g_dbg_front_baseline;
        vofa_values[3] = (float)g_dbg_rear_baseline;
        vofa_values[4] = (float)g_dbg_front_threshold;
        vofa_values[5] = (float)g_dbg_rear_threshold;
        vofa_values[6] = (float)((int32_t)g_dbg_front_prox -
                                 (int32_t)g_dbg_front_threshold);
        vofa_values[7] = (float)((int32_t)g_dbg_rear_prox -
                                 (int32_t)g_dbg_rear_threshold);
        memcpy(&vofa_frame[0], vofa_values, sizeof(vofa_values));
        memcpy(&vofa_frame[VOFA_CHANNEL_COUNT * 4U], &vofa_tail,
               sizeof(vofa_tail));
        (void)HAL_UART_Transmit(&huart2, vofa_frame, VOFA_FRAME_SIZE, 1U);
    }

    /* Debug-only sensor activity indicator: either optical sensor can light
       the upper status LED.  Keep the flag long enough for the 10 Hz LED task
       to render it even when the interrupt occurs between two task ticks. */
    uint32_t front_int_count = ShootDetect_GetFrontIntCount(&g_shoot_detect);
    uint32_t rear_int_count = ShootDetect_GetRearIntCount(&g_shoot_detect);
    if (debug_flag &&
        (front_int_count != last_led_front_int_count ||
         rear_int_count != last_led_rear_int_count)) {
        debug_sensor_flash_until = now_tick + DEBUG_SENSOR_FLASH_MS;
    }
    last_led_front_int_count = front_int_count;
    last_led_rear_int_count = rear_int_count;

    /* A confirmed shot (not merely one sensor edge) starts the 100 ms sweep. */
    uint32_t current_shot_count = ShootDetect_GetCount(&g_shoot_detect);
    gbd_shoot_count = current_shot_count;
    if (current_shot_count != last_led_shot_count) {
        last_led_shot_count = current_shot_count;
        LedStrip_StartShotEffect(now_tick);
    }

    /* Send one unsolicited 0x230 frame for every confirmed valid shot. */
    ShootEvent_t shot_event;
    while (ShootDetect_PeekEvent(&g_shoot_detect, &shot_event)) {
        if (CANProtocol_SendShotEvent(&shot_event) != HAL_OK) {
            /* Keep the event queued and retry after CAN becomes available. */
            break;
        }
        ShootDetect_DropEvent(&g_shoot_detect);
    }

    /* CAN 0x200 remains reserved for debug heartbeat but is idle in this build. */

    /* Keep raw CAN controller diagnostics visible in the debugger. */
    uint32_t can_esr = CAN->ESR;
    g_can_stats.last_error_lec = (uint8_t)((can_esr & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos);
    g_can_stats.tx_error_counter = (uint8_t)((can_esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);
    g_can_stats.rx_error_counter = (uint8_t)((can_esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos);
    g_can_stats.bus_off = (uint8_t)((can_esr & CAN_ESR_BOFF) != 0U);
    g_can_stats.state = (uint8_t)HAL_CAN_GetState(&hcan);

    /* 10 Hz tasks: status report refresh */
    if (g_sensor_tick_10hz) {
        g_sensor_tick_10hz = false;

        Reliability_ObserveSensors(g_dbg_front_prox != 0xFFFFU,
                                   g_dbg_rear_prox != 0xFFFFU);
        Reliability_ObserveEventQueueDropped(
            ShootDetect_GetDroppedEventCount(&g_shoot_detect));

        g_shoot_report.shot_count      = ShootDetect_GetCount(&g_shoot_detect);
        g_shoot_report.front_int_count = ShootDetect_GetFrontIntCount(&g_shoot_detect);
        g_shoot_report.rear_int_count  = ShootDetect_GetRearIntCount(&g_shoot_detect);
        g_shoot_report.last_speed_mps  = ShootDetect_GetLastSpeed(&g_shoot_detect);
        g_shoot_report.barrel_mask     = ShootDetect_GetState(&g_shoot_detect);
        g_shoot_report.heat_level      = g_heat_debug;
        g_shoot_report.front_prox      = g_dbg_front_prox;
        g_shoot_report.rear_prox       = g_dbg_rear_prox;
        CANProtocol_UpdateData(&g_shoot_report);
    }

    bool shot_effect_active = false;
    if (!Reliability_IsFaultAlertActive() &&
        g_led_cmd == 0 &&
        CANProtocol_GetLedCommand()->source != LED_SRC_DEBUG) {
        shot_effect_active = LedStrip_ProcessShotEffect(now_tick);
    }

    if (g_led_tick_10hz) {
        g_led_tick_10hz = false;

        bool debug_sensor_flash = debug_flag &&
            ((int32_t)(debug_sensor_flash_until - now_tick) > 0);

        /* Priority 1: any active fault overrides all ordinary LED control. */
        if (Reliability_IsFaultAlertActive()) {
            LedStrip_ShowFaultAlert(HAL_GetTick());
        }
        /* Priority 2: debugger override (g_led_cmd = 1~5) */
        else if (g_led_cmd != 0) {
            LedStrip_TestPattern(g_led_cmd, g_led_cmd_count);
        }
        /* Priority 3: CAN LED command */
        else if (CANProtocol_GetLedCommand()->source == LED_SRC_DEBUG) {
            LedStrip_ApplyCommand(CANProtocol_GetLedCommand());
        }
        /* Priority 4: valid-shot animation; it is updated every main-loop pass. */
        else if (shot_effect_active) {
            /* LedStrip_ProcessShotEffect() has already sent this frame. */
        }
        /* Priority 5: auto — follow shoot detection */
        else {
            LedStrip_SetOcclusion((g_shoot_detect.barrel_mask != 0) ||
                                  debug_sensor_flash);
            LedStrip_SetRefereeData(g_heat_debug);
            LedStrip_Update();
        }
    }

    /* Weak fault notification is best-effort and never blocks the main loop. */
    static uint8_t last_weak_mask = 0U;
    static uint32_t weak_fault_tx_tick = 0U;
    uint8_t weak_mask = Reliability_GetWeakMask();
    if (weak_mask != 0U &&
        (weak_mask != last_weak_mask ||
         (now_tick - weak_fault_tx_tick) >= 100U)) {
        (void)CANProtocol_SendWeakFault(weak_mask);
        weak_fault_tx_tick = now_tick;
    }
    last_weak_mask = weak_mask;

    /* Main-loop-only watchdog feed: do not feed it from any ISR. */
    if (!debug_flag) {
        (void)HAL_IWDG_Refresh(&hiwdg);
    }
    HAL_Delay(1);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  /* CAN bitrate: 48 MHz / 8 / (1 + 10 + 1) = 500 kbps, 12 TQ/bit. */
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

static void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 1000U;
  hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10805D88;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */
  __HAL_RCC_TIM14_CLK_ENABLE();
  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 4800-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */
  /* Reconfigure for 10 Hz update interrupt: 48 MHz / 4800 / 1000 = 10 Hz */
  TIM14->PSC = 4800 - 1;
  TIM14->ARR = 1000 - 1;
  TIM14->EGR = TIM_EGR_UG;           /* Load shadow registers              */
  TIM14->DIER |= TIM_DIER_UIE;       /* Enable update interrupt            */
  TIM14->CR1 |= TIM_CR1_CEN;         /* Start counter                      */
  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */
  __HAL_RCC_TIM15_CLK_ENABLE();
  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 4800-1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 1000-1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 500;
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
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */
  /* Override for 10 Hz LED tick: 48 MHz / 4800 / 1000 = 10 Hz */
  TIM15->PSC = 4800 - 1;
  TIM15->ARR = 1000 - 1;
  TIM15->EGR = TIM_EGR_UG;
  TIM15->DIER |= TIM_DIER_UIE;
  TIM15->CR1 |= TIM_CR1_CEN;
  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 2400-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  huart2.Init.BaudRate = VOFA_UART_BAUDRATE;
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
  huart3.Init.BaudRate = 3000000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_TXINVERT_INIT;
  huart3.AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
  if (HAL_UART_Init(&huart3) != HAL_OK)
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : IR_IIC2_INT_Pin IR_IIC1_INT_Pin */
  GPIO_InitStruct.Pin = IR_IIC2_INT_Pin|IR_IIC1_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* VCNL4040 INT open-drain → PULLUP for clean rising edge (leave event) */
  GPIO_InitStruct.Pin  = IR_IIC1_INT_Pin | IR_IIC2_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  TIM14 period elapsed callback — fires at 10 Hz.
  *         Sets a flag for the main loop to read sensors.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM14) {
        g_sensor_tick_10hz = true;
    }
    if (htim->Instance == TIM15) {
        g_led_tick_10hz = true;
    }
}

/** @brief CAN error interrupt callback; keeps cumulative diagnostics. */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan_ptr)
{
    uint32_t can_esr;

    if (hcan_ptr != &hcan) {
        return;
    }

    can_esr = CAN->ESR;
    g_can_stats.error_callbacks++;
    g_can_stats.error_events++;
    g_can_stats.error_code = HAL_CAN_GetError(hcan_ptr);
    g_can_stats.last_error_tick = HAL_GetTick();
    g_can_stats.last_error_lec = (uint8_t)((can_esr & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos);
    g_can_stats.tx_error_counter = (uint8_t)((can_esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);
    g_can_stats.rx_error_counter = (uint8_t)((can_esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos);
    g_can_stats.bus_off = (uint8_t)((can_esr & CAN_ESR_BOFF) != 0U);
    if (!debug_flag && g_can_stats.bus_off != 0U) {
        Reliability_RequestStrongFault(REL_STRONG_CAN_BUSOFF);
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
