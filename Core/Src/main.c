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
#include "thermal.h"
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

/* Debug observation variables (retained for debugger inspection only). */
volatile uint16_t g_dbg_front_prox = 0;
volatile uint16_t g_dbg_rear_prox  = 0;
/* Direct PB5/PB12 levels: true means the VCNL4040 open-drain INT is low. */
volatile bool g_dbg_front_int_pin_low = false;
volatile bool g_dbg_rear_int_pin_low  = false;
/* Peak valid PS_DATA values since this MCU boot; inspect in the debugger. */
volatile uint16_t g_dbg_front_prox_max = 0;
volatile uint16_t g_dbg_rear_prox_max  = 0;
/* Initial 0xFFFF means no valid sample has been seen since this boot. */
volatile uint16_t g_dbg_front_prox_min = 0xFFFFU;
volatile uint16_t g_dbg_rear_prox_min  = 0xFFFFU;
/* Incremented once per full front+rear I2C sample pair. */
volatile uint32_t g_dbg_sensor_sample_count = 0;
/* Measured software rate of complete front+rear polling pairs, updated each
   second.  It is not the VCNL4040 internal conversion rate. */
volatile uint32_t g_dbg_sensor_pair_rate_hz = 0;
/* Baselines captured by ShootDetect_Calibrate at this MCU boot. */
volatile uint16_t g_dbg_front_baseline = 0;
volatile uint16_t g_dbg_rear_baseline  = 0;
volatile uint16_t g_dbg_front_threshold = 0;
volatile uint16_t g_dbg_rear_threshold  = 0;
/* Debug watch variable: valid shots counted since the latest power-up/reset. */
volatile uint32_t gbd_shoot_count = 0;

/* Firmware identity and shot-to-LED trace points.
   This value is deliberately changed with this diagnostic build.  It lets the
   debugger prove which image is executing; it is not derived from the source
   timestamp. */
#if SHOOT_CAN_DIAGNOSTIC_SILENT
#define FW_BUILD_MAGIC  0x26091002UL /* CAN silent diagnostic image */
#elif SHOOT_CAN_DIAGNOSTIC_NO_APP_TX
#define FW_BUILD_MAGIC  0x26091004UL /* RX-only + reset-cause diagnostic */
#else
#define FW_BUILD_MAGIC  0x26083001UL /* normal operational image */
#endif
volatile uint32_t g_dbg_firmware_build_magic = FW_BUILD_MAGIC;
/* A mailbox accept is not yet a physical CAN ACK; it only means bxCAN took
   the 0x230 request.  Keep the two counters separate from sensor counters. */
volatile uint32_t g_dbg_shot_mailbox_accept_count = 0;
volatile uint32_t g_dbg_shot_led_start_count = 0;
volatile uint32_t g_dbg_last_shot_led_event_count = 0;

/* Keep CAN fault evidence as scalar volatile symbols.  Some GDB frontends
 * cannot create a watch for a member of CAN_ErrorStats_t, especially after
 * -Og optimisation.  These are deliberately boring debugger entry points. */
volatile uint32_t g_dbg_can_esr = 0U;
volatile uint32_t g_dbg_can_btr = 0U;
volatile uint8_t  g_dbg_can_lec = 0U;
volatile uint8_t  g_dbg_can_tec = 0U;
volatile uint8_t  g_dbg_can_rec = 0U;
volatile uint8_t  g_dbg_can_bus_off = 0U;

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
volatile uint8_t  g_heat_debug     = 0;   /* current local heat, 0–200           */
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
  bool front_sensor_ready = false;
  front_sensor_ready = (VCNL4040_Init(&hi2c1) == HAL_OK);
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
  Thermal_Init(HAL_GetTick());

  /* ---- Calibrate baseline & set thresholds ---- */
  bool sensor_calibration_ready = false;
  if (front_sensor_ready && rear_sensor_ready &&
      ShootDetect_Calibrate(&g_shoot_detect)) {
      sensor_calibration_ready = true;
  } else {
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
  LedStrip_StartBootEffect(HAL_GetTick());

  /* ---- Init CAN protocol (slave-only) ---- */
  CANProtocol_Init(&hcan);
  /* Only receive the gun command range (0x220..0x23F), standard data frames.
     Armor-board traffic never reaches this CAN RX ISR. */
  CAN_FilterTypeDef can_filter = {0};
  can_filter.FilterBank = 0;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0x4400; /* 0x220 << 5 */
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0xFC00; /* match 0x220..0x23F */
  can_filter.FilterMaskIdLow = 0x0006;  /* IDE=0, RTR=0 */
  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  can_filter.FilterActivation = ENABLE;
  can_filter.SlaveStartFilterBank = 0;
  if (HAL_CAN_ConfigFilter(&hcan, &can_filter) != HAL_OK) {
      Error_Handler();
  }
  /* Start CAN and enable receive interrupt.  The silent diagnostic image
     deliberately remains fully powered and receives 0x220..0x23F, but bxCAN
     neither ACKs nor drives any CAN bit.  It is only for isolating a
     gun-side CAN fault; production builds always use normal mode. */
  if (HAL_CAN_Start(&hcan) != HAL_OK) {
      Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan,
        CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
      Error_Handler();
  }
  g_can_stats.state = (uint8_t)HAL_CAN_GetState(&hcan);

  MX_IWDG_Init();

  /* Report a prior strong reset before announcing this successful startup. */
  uint8_t pending_strong = Reliability_GetPendingStrongMask();
  if (pending_strong != 0U) {
      (void)CANProtocol_SendStrongFault(pending_strong);
  }
  /* Keep the debugger-visible image identity linked into the final ELF. */
  if (g_dbg_firmware_build_magic != FW_BUILD_MAGIC) {
      Error_Handler();
  }
  (void)CANProtocol_SendBoot();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t sensor_rate_window_tick = HAL_GetTick();
  uint32_t sensor_rate_window_pairs = 0U;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Always check timeout + clear sensor interrupts (fast) */
    /* Do not process stale/noisy interrupt state unless both sensors completed
       initialization and calibration. Raw diagnostics remain available. */
    if (sensor_calibration_ready) {
        ShootDetect_Process(&g_shoot_detect);
    }

    /* A 0x220 request is deferred from the CAN ISR because calibration reads
       both sensors 20 times and must never block CAN interrupt handling. */
    if (CANProtocol_TakeCalibrationRequest()) {
        uint16_t old_front_baseline = g_shoot_detect.front_baseline;
        uint16_t old_rear_baseline  = g_shoot_detect.rear_baseline;
        bool calibration_ok = front_sensor_ready && rear_sensor_ready &&
                              ShootDetect_Calibrate(&g_shoot_detect);
        sensor_calibration_ready = calibration_ok;
        Reliability_SetWeakFault(REL_WEAK_SHOOT_DETECT, !calibration_ok);
        g_dbg_front_baseline  = g_shoot_detect.front_baseline;
        g_dbg_rear_baseline   = g_shoot_detect.rear_baseline;
        g_dbg_front_threshold = g_shoot_detect.front_threshold_low;
        g_dbg_rear_threshold  = g_shoot_detect.rear_threshold_low;
        (void)CANProtocol_SendCalibrationAck(
            old_front_baseline,
            calibration_ok ? g_shoot_detect.front_baseline : 0xFFFFU,
            old_rear_baseline,
            calibration_ok ? g_shoot_detect.rear_baseline : 0xFFFFU);
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
    g_dbg_front_int_pin_low =
        (HAL_GPIO_ReadPin(IR_IIC1_INT_GPIO_Port, IR_IIC1_INT_Pin) ==
         GPIO_PIN_RESET);
    g_dbg_rear_int_pin_low =
        (HAL_GPIO_ReadPin(IR_IIC2_INT_GPIO_Port, IR_IIC2_INT_Pin) ==
         GPIO_PIN_RESET);
    g_dbg_sensor_sample_count++;
    sensor_rate_window_pairs++;
    uint32_t sensor_rate_now = HAL_GetTick();
    uint32_t sensor_rate_elapsed = sensor_rate_now - sensor_rate_window_tick;
    if (sensor_rate_elapsed >= 1000U) {
        g_dbg_sensor_pair_rate_hz =
            (sensor_rate_window_pairs * 1000U) / sensor_rate_elapsed;
        sensor_rate_window_pairs = 0U;
        sensor_rate_window_tick = sensor_rate_now;
    }

    /* Keep the requested boot-calibrated baseline - 20 threshold fixed.
       A large positive reflection must not move the baseline and turn its
       departure into a false low-threshold event. */
    g_dbg_front_baseline = g_shoot_detect.front_baseline;
    g_dbg_rear_baseline  = g_shoot_detect.rear_baseline;
    g_dbg_front_threshold = g_shoot_detect.front_threshold_low;
    g_dbg_rear_threshold  = g_shoot_detect.rear_threshold_low;

    uint32_t now_tick = HAL_GetTick();
    Thermal_Update(now_tick);
    g_heat_debug = Thermal_GetHeat();
    LedStrip_SetOverheatAlert(
        Thermal_IsOverheatIndicatorActive(now_tick));

    gbd_shoot_count = ShootDetect_GetCount(&g_shoot_detect);

    /* A confirmed shot owns its local indication even when CAN is absent or
       temporarily has no free TX mailbox.  Drain the counter once; the
       queued event below remains available for later CAN transmission. */
    uint32_t shot_effect_pending =
        ShootDetect_TakeShotEffectPending(&g_shoot_detect);
    if (shot_effect_pending != 0U) {
        g_dbg_shot_led_start_count += shot_effect_pending;
        g_dbg_last_shot_led_event_count = gbd_shoot_count;
        LedStrip_StartShotEffect(now_tick);
    }

    /* A confirmed 0x230 submission is retried independently of the local
       shot indication; the rear strip stays reserved for the heat display. */
    ShootEvent_t shot_event;
    while (ShootDetect_PeekEvent(&g_shoot_detect, &shot_event)) {
        if (CANProtocol_SendShotEvent(&shot_event) != HAL_OK) {
            /* Keep the event queued and retry after CAN becomes available. */
            break;
        }
        g_dbg_shot_mailbox_accept_count++;
        ShootDetect_DropEvent(&g_shoot_detect);
    }

    /* Queries are queued by CAN RX ISR and replied from the main loop. This
       prevents an ISR reply from racing an armor-board response for CAN TX. */
    CANProtocol_Task();

    /* CAN 0x200 remains reserved for debug heartbeat but is idle in this build. */

    /* Keep raw CAN controller diagnostics visible in the debugger. */
    uint32_t can_esr = CAN->ESR;
    g_dbg_can_esr = can_esr;
    g_dbg_can_btr = CAN->BTR;
    g_dbg_can_lec = (uint8_t)((can_esr & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos);
    g_dbg_can_tec = (uint8_t)((can_esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);
    g_dbg_can_rec = (uint8_t)((can_esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos);
    g_dbg_can_bus_off = (uint8_t)((can_esr & CAN_ESR_BOFF) != 0U);
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
        /* A valid raw read alone is not enough to re-enable detection after
           startup/calibration failure. Keep this fault asserted until a
           successful calibration establishes valid thresholds. */
        if (!sensor_calibration_ready) {
            Reliability_SetWeakFault(REL_WEAK_SHOOT_DETECT, true);
        }
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

    bool boot_effect_active = LedStrip_ProcessBootEffect(now_tick);
    bool shot_effect_active = false;
    if (!boot_effect_active && !Reliability_IsFaultAlertActive() &&
        g_led_cmd == 0 && CANProtocol_GetLedCommand()->source != LED_SRC_DEBUG) {
        shot_effect_active = LedStrip_ProcessShotEffect(now_tick);
    }

    if (g_led_tick_10hz) {
        g_led_tick_10hz = false;

        /* Priority 1: one-shot power-on confirmation of the lower eight LEDs. */
        if (boot_effect_active) {
            /* LedStrip_ProcessBootEffect() has already sent this frame. */
        }
        /* Priority 2: any active fault overrides ordinary LED control. */
        else if (Reliability_IsFaultAlertActive()) {
            LedStrip_ShowFaultAlert(HAL_GetTick());
        }
        /* Priority 3: debugger override (g_led_cmd = 1~5) */
        else if (g_led_cmd != 0) {
            LedStrip_TestPattern(g_led_cmd, g_led_cmd_count);
        }
        /* Priority 4: CAN LED command */
        else if (CANProtocol_GetLedCommand()->source == LED_SRC_DEBUG) {
            LedStrip_ApplyCommand(CANProtocol_GetLedCommand());
        }
        /* Priority 5: valid-shot animation; it is updated every main-loop pass. */
        else if (shot_effect_active) {
            /* LedStrip_ProcessShotEffect() has already sent this frame. */
        }
        /* Priority 6: auto — follow shoot detection */
        else {
            LedStrip_SetRefereeData(g_heat_debug);
            LedStrip_Update();
        }
    }

    /* Weak fault notification is best-effort and never blocks the main loop. */
    static uint8_t last_weak_mask = 0U;
    static bool weak_mask_initialized = false;
    static uint32_t weak_fault_tx_tick = 0U;
    uint8_t weak_mask = Reliability_GetWeakMask();
    if ((!weak_mask_initialized && weak_mask != 0U) ||
        (weak_mask_initialized && weak_mask != last_weak_mask &&
         (now_tick - weak_fault_tx_tick) >= 100U) ||
        (weak_mask != 0U &&
         (now_tick - weak_fault_tx_tick) >= 100U)) {
        HAL_StatusTypeDef weak_tx_status =
            CANProtocol_SendWeakFault(weak_mask);
        weak_fault_tx_tick = now_tick;
        if (weak_tx_status == HAL_OK) {
            last_weak_mask = weak_mask;
        }
    }
    weak_mask_initialized = true;

    /* Main-loop-only watchdog feed: do not feed it from any ISR. */
    (void)HAL_IWDG_Refresh(&hiwdg);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
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
  /* 48 MHz / 6 / (1 + 13 + 2) = 500 kbps, sample point 87.5%.
     This matches the L431PM and all armor boards. */
  hcan.Init.Prescaler = 6;
#if SHOOT_CAN_DIAGNOSTIC_SILENT
  hcan.Init.Mode = CAN_MODE_SILENT;
#else
  hcan.Init.Mode = CAN_MODE_NORMAL;
#endif
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  /* Let bxCAN leave Bus-Off after the bus has recovered. A missing CAN
     network must not reset the whole barrel application. */
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  /* 2026-09-10：开启自动重传（NART=0）。失败帧由硬件重发；
   * 发送侧 send_frame_confirmed() 的超时路径会 abort 邮箱，避免无 ACK 帧把邮箱占死。 */
  hcan.Init.AutoRetransmission = ENABLE;
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
