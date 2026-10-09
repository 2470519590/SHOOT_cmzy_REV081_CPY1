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
#include "ir_acquisition.h"
#include "ir_detection.h"
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
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim3;

TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;
IWDG_HandleTypeDef hiwdg;

/* USER CODE BEGIN PV */
ShootData_Report_t g_shoot_report;
CAN_ErrorStats_t g_can_stats = {0};

/* 10 Hz status-report tick (set by TIM14 ISR, cleared by main loop) */
volatile bool g_sensor_tick_10hz = false;

/* 10 Hz LED refresh tick (set by TIM15 ISR, cleared by main loop) */
volatile bool g_led_tick_10hz = false;

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
/* ADC detection owns calibration readiness after its startup window. */
static bool g_boot_report_pending = true;
static uint8_t g_pending_strong_fault;
/* USER CODE END PV */



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC_Init(void);
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
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM14_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  /* ---- NVIC configuration ---- */
  Thermal_Init(HAL_GetTick());
  HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
  HAL_NVIC_SetPriority(TIM14_IRQn, 3, 0);      /* TIM14: 10 Hz sensor tick   */
  HAL_NVIC_EnableIRQ(TIM14_IRQn);
  HAL_NVIC_SetPriority(TIM15_IRQn, 3, 0);      /* TIM15: 10 Hz LED tick       */
  HAL_NVIC_EnableIRQ(TIM15_IRQn);
  HAL_NVIC_SetPriority(CEC_CAN_IRQn, 2, 0);    /* CAN: referee comms         */
  HAL_NVIC_EnableIRQ(CEC_CAN_IRQn);

  /* Start TIM16 free-running counter (PSC/ARR set by MX_TIM16_Init) */
  __HAL_TIM_ENABLE(&htim16);

  /* ---- Start fixed-code DAC emitters and timer-triggered raw acquisition ---- */
  IR_Detection_Init();
  IR_Acquisition_RegisterSampleHandler(IR_Detection_OnPhysicalSample);
  if (IR_Acquisition_Init() != HAL_OK) {
      Error_Handler();
  }
  /* The ADC detector owns the application shot path. */
  Reliability_SetWeakFault(REL_WEAK_SHOOT_DETECT, true);

  /* ---- Init board status LED (RGB_TX3) and eight-pixel strip (RGB_TX2) ---- */
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
  g_pending_strong_fault = Reliability_GetPendingStrongMask();
  /* Keep the debugger-visible image identity linked into the final ELF. */
  if (g_dbg_firmware_build_magic != FW_BUILD_MAGIC) {
      Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (CANProtocol_TakeCalibrationRequest()) {
        (void)IR_Detection_RequestCalibration();
    }

    IR_Acquisition_PublishLatest();
    IR_Acquisition_ServiceRawCapture();
    if (g_pending_strong_fault != 0U &&
        CANProtocol_SendStrongFault(g_pending_strong_fault) == HAL_OK) {
        g_pending_strong_fault = 0U;
    }
    if (g_boot_report_pending && g_pending_strong_fault == 0U &&
        IR_Detection_IsCalibrationReady() &&
        CANProtocol_SendBoot() == HAL_OK) {
        g_boot_report_pending = false;
    }

    static uint16_t old_front, new_front, old_rear, new_rear;
    static bool calibration_ack_pending;
    if (!calibration_ack_pending &&
        IR_Detection_TakeCalibrationResult(&old_front, &new_front,
                                           &old_rear, &new_rear)) {
        calibration_ack_pending = true;
    }
    if (calibration_ack_pending &&
        CANProtocol_SendCalibrationAck(old_front, new_front,
                                       old_rear, new_rear) == HAL_OK) {
        calibration_ack_pending = false;
    }

    uint32_t now_tick = HAL_GetTick();
    Thermal_Update(now_tick);
    g_heat_debug = Thermal_GetHeat();
    LedStrip_SetOverheatAlert(
        Thermal_IsOverheatIndicatorActive(now_tick));

    /* Drain acquisition events regardless of CAN availability. */
    static ShootEvent_t can_shots[IR_SHOT_EVENT_CAPACITY];
    static uint8_t can_shot_head, can_shot_count;
    static uint32_t can_shot_dropped;
    static uint32_t locally_accounted_shots;
    ShootEvent_t shot_event;
    while (IR_Detection_PeekShotEvent(&shot_event)) {
        IR_Detection_DropShotEvent();
        /* Count gaps represent detection queue loss, not lost physical shots. */
        while ((int32_t)(shot_event.shot_count - locally_accounted_shots) > 0) {
            locally_accounted_shots++;
            g_heat_debug = Thermal_AddShot(now_tick);
            g_dbg_shot_led_start_count++;
            g_dbg_last_shot_led_event_count = locally_accounted_shots;
            LedStrip_StartShotEffect(now_tick);
        }
        shot_event.heat_level = g_heat_debug;
        if (can_shot_count < IR_SHOT_EVENT_CAPACITY) {
            uint8_t tail = (uint8_t)((can_shot_head + can_shot_count) % IR_SHOT_EVENT_CAPACITY);
            can_shots[tail] = shot_event;
            can_shot_count++;
        } else {
            can_shot_dropped++;
        }
    }
    /* Preserve local behavior even when the detection event queue overflowed. */
    uint32_t shot_primask = __get_PRIMASK();
    __disable_irq();
    uint32_t detected_shots = IR_Detection_PeekShotEvent(&shot_event)
        ? locally_accounted_shots : ir_shot_count;
    __set_PRIMASK(shot_primask);
    while ((int32_t)(detected_shots - locally_accounted_shots) > 0) {
        locally_accounted_shots++;
        g_heat_debug = Thermal_AddShot(now_tick);
        g_dbg_shot_led_start_count++;
        g_dbg_last_shot_led_event_count = locally_accounted_shots;
        LedStrip_StartShotEffect(now_tick);
    }
    if (can_shot_count != 0U &&
        CANProtocol_SendShotEvent(&can_shots[can_shot_head]) == HAL_OK) {
        g_dbg_shot_mailbox_accept_count++;
        can_shot_head = (uint8_t)((can_shot_head + 1U) % IR_SHOT_EVENT_CAPACITY);
        can_shot_count--;
    }

    /* Queries are queued by CAN RX ISR and replied from the main loop. */
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

        static uint32_t last_adc_activity;
        uint32_t adc_activity = adc_activity_count;
        bool adc_alive = ir_acquisition_ready &&
                         (adc_activity != last_adc_activity);
        last_adc_activity = adc_activity;
        Reliability_ObserveSensors(adc_alive, adc_alive);
        /* A valid startup/recalibration window establishes ADC thresholds. */
        if (!IR_Detection_IsCalibrationReady()) {
            Reliability_SetWeakFault(REL_WEAK_SHOOT_DETECT, true);
        } else {
            Reliability_SetWeakFault(REL_WEAK_SHOOT_DETECT, false);
        }
        Reliability_ObserveEventQueueDropped(
            IR_Detection_GetDroppedEventCount() + can_shot_dropped);

        g_shoot_report.shot_count      = ir_shot_count;
        g_shoot_report.front_int_count = barrel_front_event_count;
        g_shoot_report.rear_int_count  = barrel_rear_event_count;
        g_shoot_report.last_speed_mps  = ir_last_speed_valid ? ir_last_speed_mps : 0.0f;
        g_shoot_report.barrel_mask     = 0U;
        g_shoot_report.heat_level      = g_heat_debug;
        g_shoot_report.front_prox      = IR_GetBarrelPhysicalFrontAdc();
        g_shoot_report.rear_prox       = IR_GetBarrelPhysicalRearAdc();
        CANProtocol_UpdateData(&g_shoot_report);
    }

    /* Feed the display model before any animation renders. Team commands
       configure the automatic strip; heat includes shots accounted above.
       The CAN ISR never calls the LED transport or renderer. */
    LedStrip_SetTeam(CANProtocol_GetLedCommand()->team == 0U
                     ? TEAM_RED : TEAM_BLUE);
    LedStrip_SetRefereeData(g_heat_debug);
    bool boot_effect_active = LedStrip_ProcessBootEffect(now_tick);
    bool shot_effect_active = false;
    if (!boot_effect_active &&
        (Reliability_GetWeakMask() & (uint8_t)~REL_WEAK_SHOOT_DETECT) == 0U &&
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
        else if ((Reliability_GetWeakMask() & (uint8_t)~REL_WEAK_SHOOT_DETECT) != 0U) {
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

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  /* RGB_TX2/PB6 carries WS2812 waveforms through the four-pin connector.
     Two LED bits per 8N1 UART frame require 4 Mbaud and idle-low TX. */
  huart1.Init.BaudRate = 4000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_TXINVERT_INIT;
  huart1.AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

/**
  * @brief DAC initialization.
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
  /* GL5050RGB01H-T uses the WS2812-style 1.25 us bit cell.  The UART
     encoder emits five UART bits per LED bit, so 4 Mbps is required. */
  huart3.Init.BaudRate = 4000000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  /* 16x oversampling would produce BRR=12 at 48 MHz/4 Mbps, which this
     STM32F0 HAL rejects (minimum BRR is 0x10).  8x keeps the same baud. */
  huart3.Init.OverSampling = UART_OVERSAMPLING_8;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Old PB5/PB12 sensor interrupts are not configured on the new board. */
  HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
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
