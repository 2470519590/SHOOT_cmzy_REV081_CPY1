#include "ir_acquisition.h"

extern DAC_HandleTypeDef hdac;

/* These are acquisition-owned implementation details. */
static volatile uint16_t adc_dma_buf[IR_ADC_DMA_LENGTH]
    __attribute__((aligned(4)));
/* 100 ms at 20 kHz, two 12-bit channels, raw DMA order [ET1, ET2]. */
volatile uint16_t ir_raw_capture_dma[IR_RAW_CAPTURE_WORDS]
    __attribute__((aligned(4)));
/* The PCB labels are ET1/PA1 on the rear physical sensor and ET2/PA3 on
   the front physical sensor.  Keep this translation here, at the hardware
   boundary; detection code only receives physical front/rear samples. */
static volatile uint16_t et1_raw; /* ET1/PA1: physical barrel rear */
static volatile uint16_t et2_raw; /* ET2/PA3: physical barrel front */
static IR_PhysicalSampleHandler sample_handler;
static uint32_t sample_number;
static uint32_t completed_sequence;
static uint16_t completed_offset;
static uint32_t published_sequence;
static uint16_t applied_dac1, applied_dac2;

volatile uint32_t adc_half_count, adc_full_count, adc_error_count;
volatile uint32_t adc_activity_count;
volatile uint16_t ir_dac1_code = IR_DAC1_DEFAULT_CODE;
volatile uint16_t ir_dac2_code = IR_DAC2_DEFAULT_CODE;
volatile bool ir_acquisition_ready;
volatile uint16_t barrel_front_adc_raw, barrel_rear_adc_raw;
volatile uint32_t ir_raw_capture_arm;
volatile uint32_t ir_raw_capture_active;
volatile uint32_t ir_raw_capture_done;
volatile uint32_t ir_raw_capture_frame_count;
volatile uint32_t ir_raw_capture_error_count;
volatile uint32_t ir_raw_capture_triggered;
volatile uint32_t ir_raw_capture_trigger_frame;
volatile uint32_t ir_raw_capture_start_word;
volatile uint32_t ir_raw_capture_trigger_index;
volatile uint32_t ir_raw_capture_valid_frames;
volatile uint16_t ir_raw_capture_baseline_front;
volatile uint16_t ir_raw_capture_baseline_rear;
volatile uint16_t ir_raw_capture_trigger_offset = 15U;

static uint32_t raw_capture_write_frame;
static uint32_t raw_capture_total_frames;
static uint32_t raw_capture_post_frames;
static uint32_t raw_capture_trigger_ring_frame;
static uint32_t raw_capture_front_sum;
static uint32_t raw_capture_rear_sum;
static bool raw_capture_finalize_pending;

static HAL_StatusTypeDef IR_Acquisition_RestartNormalSampling(void)
{
    __HAL_DMA_CLEAR_FLAG(&hdma_adc, DMA_FLAG_GL1);
    if (HAL_ADC_Start_DMA(&hadc, (uint32_t *)adc_dma_buf,
                          IR_ADC_DMA_LENGTH) != HAL_OK ||
        HAL_TIM_Base_Start(&htim3) != HAL_OK) {
        ir_acquisition_ready = false;
        adc_error_count++;
        return HAL_ERROR;
    }
    ir_acquisition_ready = true;
    return HAL_OK;
}

static uint16_t IR_ReadBarrelFrontFromDma(uint16_t offset, uint16_t pair)
{
    /* This is the only physical direction mapping in the firmware. */
    return adc_dma_buf[offset + 2U * pair + 1U]; /* ET2/PA3 */
}

static uint16_t IR_ReadBarrelRearFromDma(uint16_t offset, uint16_t pair)
{
    return adc_dma_buf[offset + 2U * pair]; /* ET1/PA1 */
}

static void IR_ForwardCompletedSamples(uint16_t offset)
{
    for (uint16_t pair = 0U; pair < IR_ADC_DMA_LENGTH / 4U; ++pair) {
        uint16_t front = IR_ReadBarrelFrontFromDma(offset, pair);
        uint16_t rear = IR_ReadBarrelRearFromDma(offset, pair);
        sample_number++;
        if (sample_handler != NULL) {
            sample_handler(front, rear, sample_number);
        }
    }
}

void IR_Acquisition_RegisterSampleHandler(IR_PhysicalSampleHandler handler)
{
    /* Registration is the only coupling point from acquisition to detection. */
    sample_handler = handler;
}

uint16_t IR_GetBarrelPhysicalFrontAdc(void)
{
    return barrel_front_adc_raw;
}

uint16_t IR_GetBarrelPhysicalRearAdc(void)
{
    return barrel_rear_adc_raw;
}

HAL_StatusTypeDef IR_Acquisition_SetDacCodes(uint16_t dac1_code,
                                             uint16_t dac2_code)
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

HAL_StatusTypeDef IR_Acquisition_Init(void)
{
    ADC_ChannelConfTypeDef channel = {0};
    DAC_ChannelConfTypeDef dac = {0};
    TIM_MasterConfigTypeDef master = {0};
    dac.DAC_Trigger = DAC_TRIGGER_NONE;
    dac.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    if (HAL_DAC_ConfigChannel(&hdac, &dac, DAC_CHANNEL_1) != HAL_OK ||
        HAL_DAC_ConfigChannel(&hdac, &dac, DAC_CHANNEL_2) != HAL_OK) return HAL_ERROR;
    if (IR_Acquisition_SetDacCodes(ir_dac1_code, ir_dac2_code) != HAL_OK) return HAL_ERROR;
    if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK ||
        HAL_DAC_Start(&hdac, DAC_CHANNEL_2) != HAL_OK) return HAL_ERROR;

    __HAL_RCC_TIM3_CLK_ENABLE();
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0U;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 2399U; /* 20 kHz ADC frame trigger. */
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
    sample_number = 0U;
    completed_sequence = 0U;
    published_sequence = 0U;
    return HAL_OK;
}

void IR_Acquisition_PublishLatest(void)
{
    uint32_t sequence = completed_sequence;
    uint16_t offset = completed_offset;
    uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_adc);
    bool safe = offset == 0U ? (remaining > 0U && remaining <= IR_ADC_DMA_LENGTH / 2U)
                             : (remaining > IR_ADC_DMA_LENGTH / 2U);
    if (ir_acquisition_ready && sequence != published_sequence && safe) {
        uint16_t front = IR_ReadBarrelFrontFromDma(offset, IR_ADC_DMA_LENGTH / 4U - 1U);
        uint16_t rear = IR_ReadBarrelRearFromDma(offset, IR_ADC_DMA_LENGTH / 4U - 1U);
        uint16_t after = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_adc);
        bool still_safe = offset == 0U ? (after > 0U && after <= IR_ADC_DMA_LENGTH / 2U)
                                       : (after > IR_ADC_DMA_LENGTH / 2U);
        if (still_safe && completed_sequence == sequence) {
            /* Preserve the PCB channel labels in the private temporaries,
               while publishing the already translated physical samples. */
            et1_raw = rear;
            et2_raw = front;
            barrel_front_adc_raw = front;
            barrel_rear_adc_raw = rear;
            published_sequence = sequence;
        }
    }
    if (applied_dac1 != ir_dac1_code || applied_dac2 != ir_dac2_code) {
        (void)IR_Acquisition_SetDacCodes(ir_dac1_code, ir_dac2_code);
    }
}

void IR_Acquisition_ServiceRawCapture(void)
{
    if (raw_capture_finalize_pending != false) {
        uint32_t start = raw_capture_trigger_ring_frame >= IR_RAW_CAPTURE_PRETRIGGER_FRAMES
            ? raw_capture_trigger_ring_frame - IR_RAW_CAPTURE_PRETRIGGER_FRAMES
            : IR_RAW_CAPTURE_FRAMES + raw_capture_trigger_ring_frame - IR_RAW_CAPTURE_PRETRIGGER_FRAMES;
        /* Keep the ring untouched. Python reorders it after the SWD read. */
        ir_raw_capture_start_word = start * 2U;
        ir_raw_capture_trigger_index = IR_RAW_CAPTURE_PRETRIGGER_FRAMES;
        ir_raw_capture_valid_frames = IR_RAW_CAPTURE_FRAMES;
        ir_raw_capture_done = 1U;
        raw_capture_finalize_pending = false;
        (void)IR_Acquisition_RestartNormalSampling();
        return;
    }
    if (ir_raw_capture_arm != 0U && ir_raw_capture_active == 0U &&
        ir_raw_capture_done == 0U) {
        (void)HAL_TIM_Base_Stop(&htim3);
        (void)HAL_ADC_Stop_DMA(&hadc);
        ir_raw_capture_frame_count = 0U;
        ir_raw_capture_triggered = 0U;
        ir_raw_capture_trigger_frame = 0U;
        ir_raw_capture_start_word = 0U;
        ir_raw_capture_trigger_index = 0U;
        ir_raw_capture_valid_frames = 0U;
        ir_raw_capture_baseline_front = 0U;
        ir_raw_capture_baseline_rear = 0U;
        raw_capture_write_frame = 0U;
        raw_capture_total_frames = 0U;
        raw_capture_post_frames = 0U;
        raw_capture_trigger_ring_frame = 0U;
        raw_capture_front_sum = 0U;
        raw_capture_rear_sum = 0U;
        raw_capture_finalize_pending = false;
        for (uint32_t i = 0U; i < IR_RAW_CAPTURE_WORDS; ++i) ir_raw_capture_dma[i] = 0U;
        ir_raw_capture_error_count = 0U;
        ir_raw_capture_active = 1U;
        __HAL_DMA_CLEAR_FLAG(&hdma_adc, DMA_FLAG_GL1);
        if (HAL_ADC_Start_DMA(&hadc, (uint32_t *)adc_dma_buf,
                              IR_ADC_DMA_LENGTH) != HAL_OK ||
            HAL_TIM_Base_Start(&htim3) != HAL_OK) {
            ir_raw_capture_active = 0U;
            ir_raw_capture_error_count++;
            ir_raw_capture_arm = 0U;
        }
    }
}

static void IR_RawCaptureForwardSamples(uint16_t offset)
{
    for (uint16_t pair = 0U; pair < IR_ADC_DMA_LENGTH / 4U; ++pair) {
        uint16_t et1 = adc_dma_buf[offset + 2U * pair];
        uint16_t et2 = adc_dma_buf[offset + 2U * pair + 1U];
        uint32_t frame = raw_capture_write_frame;
        ir_raw_capture_dma[2U * frame] = et1;
        ir_raw_capture_dma[2U * frame + 1U] = et2;
        raw_capture_write_frame = (frame + 1U) % IR_RAW_CAPTURE_FRAMES;
        raw_capture_total_frames++;
        if (raw_capture_total_frames <= IR_RAW_CAPTURE_BASELINE_FRAMES) {
            raw_capture_front_sum += et2;
            raw_capture_rear_sum += et1;
            if (raw_capture_total_frames == IR_RAW_CAPTURE_BASELINE_FRAMES) {
                ir_raw_capture_baseline_front =
                    (uint16_t)(raw_capture_front_sum / IR_RAW_CAPTURE_BASELINE_FRAMES);
                ir_raw_capture_baseline_rear =
                    (uint16_t)(raw_capture_rear_sum / IR_RAW_CAPTURE_BASELINE_FRAMES);
            }
        }
        if (ir_raw_capture_triggered == 0U &&
            raw_capture_total_frames >= IR_RAW_CAPTURE_PRETRIGGER_FRAMES &&
            raw_capture_total_frames >= IR_RAW_CAPTURE_BASELINE_FRAMES &&
            (et2 >= (uint32_t)ir_raw_capture_baseline_front + ir_raw_capture_trigger_offset ||
             et1 >= (uint32_t)ir_raw_capture_baseline_rear + ir_raw_capture_trigger_offset)) {
            ir_raw_capture_triggered = 1U;
            ir_raw_capture_trigger_frame = raw_capture_total_frames;
            raw_capture_trigger_ring_frame = frame;
            /* The trigger sample itself is post-trigger sample 0. */
            raw_capture_post_frames = 1U;
        } else if (ir_raw_capture_triggered != 0U) {
            raw_capture_post_frames++;
            if (raw_capture_post_frames >= IR_RAW_CAPTURE_POSTTRIGGER_FRAMES) {
                (void)HAL_TIM_Base_Stop(&htim3);
                (void)HAL_ADC_Stop_DMA(&hadc);
                ir_raw_capture_frame_count = IR_RAW_CAPTURE_FRAMES;
                ir_raw_capture_active = 0U;
                ir_raw_capture_arm = 0U;
                raw_capture_finalize_pending = true;
                return;
            }
        }
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *handle)
{
    if (handle == &hadc) {
        adc_half_count++;
        adc_activity_count++;
        if (ir_raw_capture_active != 0U) {
            IR_RawCaptureForwardSamples(0U);
            return;
        }
        completed_offset = 0U;
        completed_sequence++;
        IR_ForwardCompletedSamples(0U);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *handle)
{
    if (handle == &hadc) {
        adc_full_count++;
        adc_activity_count++;
        if (ir_raw_capture_active != 0U) {
            IR_RawCaptureForwardSamples(IR_ADC_DMA_LENGTH / 2U);
            return;
        }
        completed_offset = IR_ADC_DMA_LENGTH / 2U;
        completed_sequence++;
        IR_ForwardCompletedSamples(IR_ADC_DMA_LENGTH / 2U);
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *handle)
{
    if (handle == &hadc) {
        adc_error_count++;
        if (ir_raw_capture_active != 0U) {
            ir_raw_capture_active = 0U;
            ir_raw_capture_arm = 0U;
            ir_raw_capture_error_count++;
        }
        ir_acquisition_ready = false;
    }
}
