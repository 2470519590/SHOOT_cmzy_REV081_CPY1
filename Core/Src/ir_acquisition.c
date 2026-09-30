#include "ir_acquisition.h"

extern DAC_HandleTypeDef hdac;
volatile uint16_t adc_dma_buf[IR_ADC_DMA_LENGTH] __attribute__((aligned(4)));
volatile uint16_t et1_raw;
volatile uint16_t et2_raw;
volatile uint32_t adc_half_count;
volatile uint32_t adc_full_count;
volatile uint32_t adc_error_count;
volatile uint16_t ir_dac1_code = IR_DAC1_DEFAULT_CODE;
volatile uint16_t ir_dac2_code = IR_DAC2_DEFAULT_CODE;
volatile bool ir_acquisition_ready;
volatile bool g_sensor_detection_ready;
volatile uint32_t ir_rear_event_count;
volatile uint32_t ir_front_event_count;
volatile uint32_t ir_rear_event_sample;
volatile uint32_t ir_front_event_sample;
volatile bool ir_rear_event_pending;
volatile bool ir_front_event_pending;
static volatile uint32_t completed_sequence;
static volatile uint16_t completed_offset;
static uint32_t published_sequence;
static uint16_t applied_dac1;
static uint16_t applied_dac2;
static bool baseline_ready;
static bool rear_armed;
static bool front_armed;
static uint32_t calibration_start_tick;
static uint32_t calibration_sample_count;
/* 8-bit bins retain enough precision for a +45-code trigger while keeping
   the startup calibration footprint small on the 16 KiB SRAM part. */
static uint16_t rear_histogram[256];
static uint16_t front_histogram[256];
static int32_t rear_baseline_q16;
static int32_t front_baseline_q16;

/* These debugger-visible values are owned by main.c.  The first completed
   DMA block establishes the static level; subsequent samples use +45. */
extern volatile uint16_t g_dbg_rear_baseline;
extern volatile uint16_t g_dbg_front_baseline;
extern volatile uint16_t g_dbg_rear_threshold;
extern volatile uint16_t g_dbg_front_threshold;

static uint16_t IR_HistogramMedian(const uint16_t *histogram, uint32_t total)
{
    uint32_t accumulated = 0U;
    uint16_t value;
    for (value = 0U; value < 256U; ++value) {
        accumulated += histogram[value];
        if (accumulated >= (total + 1U) / 2U) {
            return value;
        }
    }
    return 0U;
}

static void IR_FinishStartupCalibration(void)
{
    uint16_t rear = (uint16_t)(IR_HistogramMedian(rear_histogram, calibration_sample_count) * 16U + 8U);
    uint16_t front = (uint16_t)(IR_HistogramMedian(front_histogram, calibration_sample_count) * 16U + 8U);
    rear_baseline_q16 = (int32_t)rear << 16;
    front_baseline_q16 = (int32_t)front << 16;
    /* Legacy debug names are reversed on this board: rear variable is front
       sensor (ET2), front variable is rear sensor (ET1). */
    g_dbg_rear_baseline = front;
    g_dbg_front_baseline = rear;
    g_dbg_rear_threshold = (uint16_t)(front + IR_TRIGGER_DELTA > 4095U ?
                                      4095U : front + IR_TRIGGER_DELTA);
    g_dbg_front_threshold = (uint16_t)(rear + IR_TRIGGER_DELTA > 4095U ?
                                       4095U : rear + IR_TRIGGER_DELTA);
    baseline_ready = true;
    rear_armed = true;
    front_armed = true;
    g_sensor_detection_ready = true;
}

static void IR_ScanCompletedBlock(uint16_t offset)
{
    const uint16_t pairs = IR_ADC_DMA_LENGTH / 4U;
    uint16_t i;

    if (!baseline_ready) {
        for (i = 0U; i < pairs; ++i) {
            uint16_t front = adc_dma_buf[offset + (uint16_t)(2U * i)];
            uint16_t rear = adc_dma_buf[offset + (uint16_t)(2U * i + 1U)];
            if (front_histogram[front >> 4] != 0xFFFFU) {
                front_histogram[front >> 4]++;
            }
            if (rear_histogram[rear >> 4] != 0xFFFFU) {
                rear_histogram[rear >> 4]++;
            }
            calibration_sample_count++;
        }
        if ((HAL_GetTick() - calibration_start_tick) >= IR_STARTUP_CALIBRATION_MS) {
            IR_FinishStartupCalibration();
        }
        return;
    }

    for (i = 0U; i < pairs; ++i) {
        uint32_t sample_index = (uint32_t)offset + (uint32_t)(2U * i);
        uint16_t front = adc_dma_buf[sample_index];
        uint16_t rear = adc_dma_buf[sample_index + 1U];
        uint16_t rear_baseline = (uint16_t)(rear_baseline_q16 >> 16);
        uint16_t front_baseline = (uint16_t)(front_baseline_q16 >> 16);
        uint16_t rear_threshold = (uint16_t)(rear_baseline + IR_TRIGGER_DELTA > 4095U ?
                                             4095U : rear_baseline + IR_TRIGGER_DELTA);
        uint16_t front_threshold = (uint16_t)(front_baseline + IR_TRIGGER_DELTA > 4095U ?
                                              4095U : front_baseline + IR_TRIGGER_DELTA);
        g_dbg_rear_baseline = front_baseline;
        g_dbg_front_baseline = rear_baseline;
        g_dbg_rear_threshold = front_threshold;
        g_dbg_front_threshold = rear_threshold;
        if (front >= front_threshold) {
            if (front_armed) {
                front_armed = false;
                ir_front_event_count++;
                ir_front_event_sample = sample_index;
                ir_front_event_pending = true;
            }
        } else {
            front_armed = true;
        }
        if (rear >= rear_threshold) {
            if (rear_armed) {
                rear_armed = false;
                ir_rear_event_count++;
                ir_rear_event_sample = sample_index;
                ir_rear_event_pending = true;
            }
        } else {
            rear_armed = true;
        }
        /* Q16 prevents small environmental changes from being lost to
           integer truncation.  The slow EMA follows ambient drift while a
           short projectile pulse changes the baseline only negligibly. */
        front_baseline_q16 += (((int32_t)front << 16) - front_baseline_q16) /
                              IR_BASELINE_TRACK_DIVISOR;
        rear_baseline_q16 += (((int32_t)rear << 16) - rear_baseline_q16) /
                             IR_BASELINE_TRACK_DIVISOR;
    }
}

HAL_StatusTypeDef IR_Acquisition_Init(void)
{
    ADC_ChannelConfTypeDef channel = {0};
    DAC_ChannelConfTypeDef dac = {0};
    TIM_MasterConfigTypeDef master = {0};
    dac.DAC_Trigger = DAC_TRIGGER_NONE;
    dac.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    if (HAL_DAC_ConfigChannel(&hdac, &dac, DAC_CHANNEL_1) != HAL_OK ||
        HAL_DAC_ConfigChannel(&hdac, &dac, DAC_CHANNEL_2) != HAL_OK) return HAL_ERROR;
    if (IR_Acquisition_SetDacCodes(ir_dac1_code, ir_dac2_code) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK ||
        HAL_DAC_Start(&hdac, DAC_CHANNEL_2) != HAL_OK) return HAL_ERROR;

    __HAL_RCC_TIM3_CLK_ENABLE();
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0U;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 2399U; /* 48 MHz / 2400 = 20 kHz per channel. */
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) return HAL_ERROR;
    master.MasterOutputTrigger = TIM_TRGO_UPDATE;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &master) != HAL_OK) return HAL_ERROR;

    hadc.Instance = ADC1;
    hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
    hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc.Init.LowPowerAutoWait = DISABLE;
    hadc.Init.LowPowerAutoPowerOff = DISABLE;
    hadc.Init.ContinuousConvMode = DISABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc.Init.DMAContinuousRequests = ENABLE;
    hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    if (HAL_ADC_Init(&hadc) != HAL_OK) return HAL_ERROR;
    /* 2 * (239.5 + 12.5) / 12 MHz = 42 us, below the 50 us trigger period. */
    channel.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    channel.Rank = ADC_RANK_CHANNEL_NUMBER;
    channel.Channel = ADC_CHANNEL_1;
    if (HAL_ADC_ConfigChannel(&hadc, &channel) != HAL_OK) return HAL_ERROR;
    channel.Channel = ADC_CHANNEL_3;
    if (HAL_ADC_ConfigChannel(&hadc, &channel) != HAL_OK) return HAL_ERROR;
    if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK) return HAL_ERROR;
    if (HAL_ADC_Start_DMA(&hadc, (uint32_t *)adc_dma_buf, IR_ADC_DMA_LENGTH) != HAL_OK) return HAL_ERROR;
    if (HAL_TIM_Base_Start(&htim3) != HAL_OK) return HAL_ERROR;
    ir_acquisition_ready = true;
    g_sensor_detection_ready = false;
    baseline_ready = false;
    rear_armed = false;
    front_armed = false;
    calibration_start_tick = HAL_GetTick();
    calibration_sample_count = 0U;
    for (uint16_t i = 0U; i < 256U; ++i) {
        rear_histogram[i] = 0U;
        front_histogram[i] = 0U;
    }
    ir_rear_event_pending = false;
    ir_front_event_pending = false;
    return HAL_OK;
}

HAL_StatusTypeDef IR_Acquisition_SetDacCodes(uint16_t dac1_code, uint16_t dac2_code)
{
    ir_dac1_code = dac1_code > 4095U ? 4095U : dac1_code;
    ir_dac2_code = dac2_code > 4095U ? 4095U : dac2_code;
    if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, ir_dac1_code) != HAL_OK ||
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, ir_dac2_code) != HAL_OK) {
        adc_error_count++;
        ir_acquisition_ready = false;
        return HAL_ERROR;
    }
    applied_dac1 = ir_dac1_code;
    applied_dac2 = ir_dac2_code;
    return HAL_OK;
}

void IR_Acquisition_PublishLatest(void)
{
    uint32_t sequence = completed_sequence;
    uint16_t offset = completed_offset;
    uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_adc);
    /* Never read a half that DMA has already begun overwriting. */
    bool half_safe = offset == 0U ? (remaining > 0U && remaining <= IR_ADC_DMA_LENGTH / 2U)
                                 : (remaining > IR_ADC_DMA_LENGTH / 2U);
    if (ir_acquisition_ready && sequence != published_sequence && half_safe) {
        uint16_t first = adc_dma_buf[offset + IR_ADC_DMA_LENGTH / 2U - 2U];
        uint16_t second = adc_dma_buf[offset + IR_ADC_DMA_LENGTH / 2U - 1U];
        uint16_t after = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_adc);
        bool still_safe = offset == 0U ? (after > 0U && after <= IR_ADC_DMA_LENGTH / 2U)
                                      : (after > IR_ADC_DMA_LENGTH / 2U);
        if (still_safe && completed_sequence == sequence) {
            et1_raw = first;
            et2_raw = second;
            published_sequence = sequence;
        }
    }
    if (applied_dac1 != ir_dac1_code || applied_dac2 != ir_dac2_code) {
        (void)IR_Acquisition_SetDacCodes(ir_dac1_code, ir_dac2_code);
    }
}

bool IR_Acquisition_TakeRearEvent(void)
{
    bool pending = ir_rear_event_pending;
    if (pending) {
        ir_rear_event_pending = false;
    }
    return pending;
}

bool IR_Acquisition_TakeFrontEvent(void)
{
    bool pending = ir_front_event_pending;
    if (pending) {
        ir_front_event_pending = false;
    }
    return pending;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *handle)
{
    if (handle == &hadc) {
        adc_half_count++;
        completed_offset = 0U;
        completed_sequence++;
        IR_ScanCompletedBlock(0U);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *handle)
{
    if (handle == &hadc) {
        adc_full_count++;
        completed_offset = IR_ADC_DMA_LENGTH / 2U;
        completed_sequence++;
        IR_ScanCompletedBlock(IR_ADC_DMA_LENGTH / 2U);
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *handle)
{
    if (handle == &hadc) {
        adc_error_count++;
        ir_acquisition_ready = false;
    }
}
