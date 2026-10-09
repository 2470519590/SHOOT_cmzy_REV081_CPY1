#ifndef IR_ACQUISITION_H
#define IR_ACQUISITION_H

#include "stm32f0xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define IR_ADC_DMA_LENGTH 32U
#define IR_RAW_CAPTURE_FRAMES 2000U
#define IR_RAW_CAPTURE_WORDS (IR_RAW_CAPTURE_FRAMES * 2U)
#define IR_RAW_CAPTURE_PRETRIGGER_FRAMES 500U
#define IR_RAW_CAPTURE_POSTTRIGGER_FRAMES 1500U
#define IR_RAW_CAPTURE_BASELINE_FRAMES 200U
#define IR_DAC1_DEFAULT_CODE 1550U
#define IR_DAC2_DEFAULT_CODE 1550U

/* Acquisition owns ADC/DMA order and physical mapping: ET1/PA1 is rear,
   ET2/PA3 is front. Detection receives named physical samples only. */
typedef void (*IR_PhysicalSampleHandler)(uint16_t barrel_front_adc,
                                         uint16_t barrel_rear_adc,
                                         uint32_t sample_number);

extern ADC_HandleTypeDef hadc;
extern DMA_HandleTypeDef hdma_adc;
extern TIM_HandleTypeDef htim3;
extern volatile uint32_t adc_half_count, adc_full_count, adc_error_count;
extern volatile uint32_t adc_activity_count;
extern volatile uint16_t ir_dac1_code, ir_dac2_code;
extern volatile bool ir_acquisition_ready;
extern volatile uint16_t barrel_front_adc_raw, barrel_rear_adc_raw;
/* SWD-controlled raw capture. Buffer order is raw ADC scan order [ET1, ET2]. */
extern volatile uint16_t ir_raw_capture_dma[IR_RAW_CAPTURE_WORDS];
extern volatile uint32_t ir_raw_capture_arm;
extern volatile uint32_t ir_raw_capture_active;
extern volatile uint32_t ir_raw_capture_done;
extern volatile uint32_t ir_raw_capture_frame_count;
extern volatile uint32_t ir_raw_capture_error_count;
extern volatile uint32_t ir_raw_capture_triggered;
extern volatile uint32_t ir_raw_capture_trigger_frame;
extern volatile uint32_t ir_raw_capture_start_word;
extern volatile uint32_t ir_raw_capture_trigger_index;
extern volatile uint32_t ir_raw_capture_valid_frames;
extern volatile uint16_t ir_raw_capture_baseline_front;
extern volatile uint16_t ir_raw_capture_baseline_rear;
extern volatile uint16_t ir_raw_capture_trigger_offset;

HAL_StatusTypeDef IR_Acquisition_Init(void);
HAL_StatusTypeDef IR_Acquisition_SetDacCodes(uint16_t dac1_code,
                                             uint16_t dac2_code);
void IR_Acquisition_RegisterSampleHandler(IR_PhysicalSampleHandler handler);
uint16_t IR_GetBarrelPhysicalFrontAdc(void);
uint16_t IR_GetBarrelPhysicalRearAdc(void);
void IR_Acquisition_PublishLatest(void);
void IR_Acquisition_ServiceRawCapture(void);

#endif
