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
/* Physical names: front=ET1/PA1, rear=ET2/PA3. Updated from every DMA
   sample, independently of detection thresholds and main-loop publishing. */
volatile uint16_t ir_front_raw_min = 4095U;
volatile uint16_t ir_front_raw_max;
volatile uint16_t ir_rear_raw_min = 4095U;
volatile uint16_t ir_rear_raw_max;
volatile uint16_t ir_front_raw_reference;
volatile uint16_t ir_rear_raw_reference;
volatile uint32_t ir_extrema_sample_count;
volatile uint32_t ir_extrema_reset_count;
volatile bool ir_extrema_reset_request = true;
volatile bool ir_extrema_hold;
/* Running maxima in automatically restarting 5 s windows. */
volatile uint16_t ir_front_adc_max_5s;
volatile uint16_t ir_rear_adc_max_5s;
static uint32_t adc_max_window_tick;
/* All times below use the 20 kHz per-channel ADC frame clock. */
volatile uint16_t ir_window_samples = 8U;
volatile uint16_t ir_front_pp_threshold = 30U, ir_rear_pp_threshold = 30U;
volatile uint16_t ir_front_level_threshold = 30U, ir_rear_level_threshold = 30U;
volatile uint16_t ir_confirm_samples = 2U, ir_release_percent = 60U;
volatile uint32_t ir_quiet_ms = 80U, ir_pair_timeout_ms = 2000U;
volatile uint32_t ir_pair_min_us = 500U;
volatile uint32_t ir_rebase_after_ms = 1000U;
volatile uint16_t ir_distance_mm = 50U;
volatile uint16_t ir_front_pp, ir_rear_pp;
volatile uint16_t ir_front_pp_max_5s, ir_rear_pp_max_5s;
volatile bool ir_front_active, ir_rear_active;
volatile uint32_t ir_shot_count, ir_pair_timeout_count, ir_pair_dropped_count;
volatile uint32_t ir_unpaired_front_count, ir_last_pair_us, ir_last_speed_mm_s;
static uint32_t sample_tick;
static uint32_t pair_ticks[8];
static uint8_t pair_head, pair_tail;
typedef struct {
    uint16_t history[16];
    uint8_t position, filled;
    uint16_t confirm;
    uint32_t quiet;
    uint32_t age;
    bool active;
} IR_Detector;
static IR_Detector front_detector, rear_detector;

static void IR_PairEvent(bool front)
{
    if (!front) {
        uint8_t next = (uint8_t)((pair_head + 1U) % 8U);
        if (next == pair_tail) { ir_pair_dropped_count++; return; }
        pair_ticks[pair_head] = sample_tick;
        pair_head = next;
    } else if (pair_head != pair_tail) {
        uint32_t delta = sample_tick - pair_ticks[pair_tail];
        uint32_t minimum = ir_pair_min_us > 1000000U ? 1000000U : ir_pair_min_us;
        if (delta == 0U || delta < (minimum + 49U) / 50U) {
            ir_unpaired_front_count++;
            return;
        }
        pair_tail = (uint8_t)((pair_tail + 1U) % 8U);
        ir_last_pair_us = delta * 50U;
        ir_last_speed_mm_s = (uint32_t)ir_distance_mm * 20000U / delta;
        ir_shot_count++;
    } else { ir_unpaired_front_count++; }
}

static bool IR_DetectSample(IR_Detector *d, uint16_t value,
                            uint16_t baseline, bool front)
{
    uint16_t n = ir_window_samples;
    if (n < 2U) n = 2U;
    if (n > 16U) n = 16U;
    d->history[d->position] = value;
    d->position = (uint8_t)((d->position + 1U) % 16U);
    if (d->filled < 16U) d->filled++;
    uint16_t low = value, high = value;
    uint16_t available = d->filled < n ? d->filled : n;
    for (uint16_t j = 1U; j < available; ++j) {
        uint16_t v = d->history[(d->position + 15U - j) % 16U];
        if (v < low) low = v;
        if (v > high) high = v;
    }
    uint16_t pp = high - low;
    uint16_t deviation = value > baseline ? value - baseline : baseline - value;
    uint16_t pp_limit = front ? ir_front_pp_threshold : ir_rear_pp_threshold;
    uint16_t level_limit = front ? ir_front_level_threshold : ir_rear_level_threshold;
    if (pp_limit == 0U) pp_limit = 1U;
    if (level_limit == 0U) level_limit = 1U;
    uint16_t release = ir_release_percent;
    if (release < 1U) release = 1U;
    if (release > 99U) release = 99U;
    if (front) {
        ir_front_pp = pp;
        if (pp > ir_front_pp_max_5s) ir_front_pp_max_5s = pp;
    } else {
        ir_rear_pp = pp;
        if (pp > ir_rear_pp_max_5s) ir_rear_pp_max_5s = pp;
    }
    bool event = false;
    if (!d->active) {
        if (pp >= pp_limit || deviation >= level_limit) {
            if (d->confirm < 65535U) d->confirm++;
            uint16_t required = ir_confirm_samples == 0U ? 1U : ir_confirm_samples;
            if (d->confirm >= required) {
                d->active = true;
                d->quiet = 0U;
                d->age = 0U;
                event = true;
            }
        } else { d->confirm = 0U; }
    } else {
        if (d->age < 0xFFFFFFFFU) d->age++;
        if ((uint32_t)pp * 100U < (uint32_t)pp_limit * release &&
            (uint32_t)deviation * 100U < (uint32_t)level_limit * release) {
            if (d->quiet < 0xFFFFFFFFU) d->quiet++;
            uint32_t quiet_ms = ir_quiet_ms > 10000U ? 10000U : ir_quiet_ms;
            if (d->quiet >= (quiet_ms == 0U ? 1U : quiet_ms * 20U)) {
                d->active = false;
                d->confirm = 0U;
            }
        } else { d->quiet = 0U; }
    }
    if (front) ir_front_active = d->active;
    else ir_rear_active = d->active;
    return event;
}
static volatile uint32_t completed_sequence;
static volatile uint16_t completed_offset;
static uint32_t published_sequence;
static uint16_t applied_dac1;
static uint16_t applied_dac2;
static bool baseline_ready;
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
       sensor (ET1), front variable is rear sensor (ET2). */
    g_dbg_rear_baseline = front;
    g_dbg_front_baseline = rear;
    g_dbg_rear_threshold = (uint16_t)(front + IR_TRIGGER_DELTA > 4095U ?
                                      4095U : front + IR_TRIGGER_DELTA);
    g_dbg_front_threshold = (uint16_t)(rear + IR_TRIGGER_DELTA > 4095U ?
                                       4095U : rear + IR_TRIGGER_DELTA);
    baseline_ready = true;
    g_sensor_detection_ready = true;
}

static void IR_ScanCompletedBlock(uint16_t offset)
{
    const uint16_t pairs = IR_ADC_DMA_LENGTH / 4U;
    uint16_t i;
    uint32_t window_tick = HAL_GetTick();
    if ((uint32_t)(window_tick - adc_max_window_tick) >= 5000U) {
        ir_front_adc_max_5s = 0U;
        ir_rear_adc_max_5s = 0U;
        ir_front_pp_max_5s = 0U;
        ir_rear_pp_max_5s = 0U;
        adc_max_window_tick = window_tick;
    }
    for (i = 0U; i < pairs; ++i) {
        uint16_t front = adc_dma_buf[offset + 2U * i];
        uint16_t rear = adc_dma_buf[offset + 2U * i + 1U];
        if (front > ir_front_adc_max_5s) ir_front_adc_max_5s = front;
        if (rear > ir_rear_adc_max_5s) ir_rear_adc_max_5s = rear;
    }

    /* Reset is acknowledged by the sole writer (DMA ISR), so a debugger
       reset cannot race individual min/max writes. Reference stays fixed. */
    if (ir_extrema_reset_request) {
        ir_front_raw_min = 4095U;
        ir_front_raw_max = 0U;
        ir_rear_raw_min = 4095U;
        ir_rear_raw_max = 0U;
        ir_front_raw_reference = adc_dma_buf[offset];
        ir_rear_raw_reference = adc_dma_buf[offset + 1U];
        ir_extrema_sample_count = 0U;
        ir_extrema_reset_count++;
        ir_extrema_reset_request = false;
    }
    if (!ir_extrema_hold) {
        for (i = 0U; i < pairs; ++i) {
            uint16_t front = adc_dma_buf[offset + 2U * i];
            uint16_t rear = adc_dma_buf[offset + 2U * i + 1U];
            if (front < ir_front_raw_min) ir_front_raw_min = front;
            if (front > ir_front_raw_max) ir_front_raw_max = front;
            if (rear < ir_rear_raw_min) ir_rear_raw_min = rear;
            if (rear > ir_rear_raw_max) ir_rear_raw_max = rear;
        }
        ir_extrema_sample_count += pairs;
    }

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
        sample_tick++;
        uint32_t timeout = ir_pair_timeout_ms > 60000U ? 60000U : ir_pair_timeout_ms;
        while (pair_tail != pair_head && sample_tick - pair_ticks[pair_tail] > timeout * 20U) {
            pair_tail = (uint8_t)((pair_tail + 1U) % 8U);
            ir_pair_timeout_count++;
        }
        if (IR_DetectSample(&rear_detector, rear, rear_baseline, false)) {
            ir_rear_event_count++;
            ir_rear_event_sample = sample_tick;
            ir_rear_event_pending = true;
            IR_PairEvent(false);
        }
        if (IR_DetectSample(&front_detector, front, front_baseline, true)) {
            ir_front_event_count++;
            ir_front_event_sample = sample_tick;
            ir_front_event_pending = true;
            IR_PairEvent(true);
        }
        /* Q16 prevents small environmental changes from being lost to
           integer truncation.  The slow EMA follows ambient drift while a
           short projectile pulse changes the baseline only negligibly. */
        uint32_t rebase_ms = ir_rebase_after_ms > 60000U ? 60000U : ir_rebase_after_ms;
        if ((!front_detector.active && front_detector.confirm == 0U) ||
            (rebase_ms != 0U && front_detector.age >= rebase_ms * 20U))
            front_baseline_q16 += (((int32_t)front << 16) - front_baseline_q16) /
                              IR_BASELINE_TRACK_DIVISOR;
        if ((!rear_detector.active && rear_detector.confirm == 0U) ||
            (rebase_ms != 0U && rear_detector.age >= rebase_ms * 20U))
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
