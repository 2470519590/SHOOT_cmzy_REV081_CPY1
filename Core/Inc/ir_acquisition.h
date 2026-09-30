#ifndef IR_ACQUISITION_H
#define IR_ACQUISITION_H

#include "stm32f0xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define IR_ADC_DMA_LENGTH 32U
#define IR_DAC1_DEFAULT_CODE 1550U
#define IR_DAC2_DEFAULT_CODE 1550U
#define IR_TRIGGER_DELTA 45U
#define IR_STARTUP_CALIBRATION_MS 1000U
#define IR_BASELINE_TRACK_DIVISOR 2048

extern ADC_HandleTypeDef hadc;
extern DMA_HandleTypeDef hdma_adc;
extern TIM_HandleTypeDef htim3;

extern volatile uint16_t adc_dma_buf[IR_ADC_DMA_LENGTH];
extern volatile uint16_t et1_raw;
extern volatile uint16_t et2_raw;
extern volatile uint32_t adc_half_count;
extern volatile uint32_t adc_full_count;
extern volatile uint32_t adc_error_count;
extern volatile uint16_t ir_dac1_code;
extern volatile uint16_t ir_dac2_code;
extern volatile bool ir_acquisition_ready;
extern volatile bool g_sensor_detection_ready;
extern volatile uint32_t ir_rear_event_count;
extern volatile uint32_t ir_front_event_count;
extern volatile uint32_t ir_rear_event_sample;
extern volatile uint32_t ir_front_event_sample;
extern volatile bool ir_rear_event_pending;
extern volatile bool ir_front_event_pending;

HAL_StatusTypeDef IR_Acquisition_Init(void);
HAL_StatusTypeDef IR_Acquisition_SetDacCodes(uint16_t dac1_code, uint16_t dac2_code);
void IR_Acquisition_PublishLatest(void);
bool IR_Acquisition_TakeRearEvent(void);
bool IR_Acquisition_TakeFrontEvent(void);

#endif
