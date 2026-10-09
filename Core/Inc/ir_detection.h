#ifndef IR_DETECTION_H
#define IR_DETECTION_H

#include <stdint.h>
#include <stdbool.h>

#define IR_ADC_SAMPLE_US                 50U
#define IR_STARTUP_CALIBRATION_SAMPLES   20000U /* 1 s at 20 kHz/channel */
#define IR_BASELINE_TRACK_DIVISOR        2048
#define IR_FRONT_TRIGGER_OFFSET_DEFAULT  60U
#define IR_REAR_TRIGGER_OFFSET_DEFAULT   18U
#define IR_PULSE_MAX_US                  6000U
#define IR_REAR_PULSE_MIN_US             300U
#define IR_DISTANCE_MM_DEFAULT           50U
#define IR_SPEED_MIN_MPS                 2.0f
#define IR_SPEED_MAX_MPS                 15.0f
#define IR_PEAK_DELTA_MIN_US             3333U
#define IR_PEAK_DELTA_MAX_US             25000U
#define IR_REAR_CANDIDATE_CAPACITY       8U
#define IR_SHOT_EVENT_CAPACITY           8U

/* One front pulse is one shot. Rear is an optional speed candidate. */
typedef struct {
    uint32_t shot_count;
    float speed_mps;
    bool speed_valid;
    uint32_t peak_delta_us;
    uint8_t barrel_mask; /* New ADC detector has no old FIFO state: always 0. */
    uint8_t heat_level;
} ShootEvent_t;

/* Required Ozone-visible physical diagnostics. */
extern volatile uint16_t barrel_front_adc_raw, barrel_rear_adc_raw;
extern volatile uint16_t barrel_front_baseline, barrel_rear_baseline;
extern volatile uint16_t barrel_front_trigger_level, barrel_rear_trigger_level;
extern volatile uint32_t barrel_front_event_count, barrel_rear_event_count;
extern volatile uint32_t barrel_front_event_sample, barrel_rear_event_sample;
extern volatile uint16_t barrel_front_peak_adc, barrel_rear_peak_adc;
extern volatile uint32_t barrel_front_peak_sample, barrel_rear_peak_sample;
extern volatile uint32_t barrel_front_long_pulse_count, barrel_rear_long_pulse_count;
extern volatile uint32_t barrel_front_last_pulse_us, barrel_rear_last_pulse_us;
extern volatile uint32_t ir_shot_count, ir_pair_timeout_count;
extern volatile uint32_t ir_unpaired_front_count, ir_unpaired_rear_count;
extern volatile float ir_last_speed_mps;
extern volatile bool ir_last_speed_valid;
extern volatile uint32_t ir_last_peak_delta_us;
extern volatile uint16_t ir_distance_mm;
extern volatile bool barrel_front_active, barrel_rear_active;

/* Existing raw-observation/SWD diagnostics retained. */
extern volatile uint16_t barrel_front_raw_min, barrel_front_raw_max;
extern volatile uint16_t barrel_rear_raw_min, barrel_rear_raw_max;
extern volatile uint16_t barrel_front_raw_reference, barrel_rear_raw_reference;
extern volatile uint32_t ir_extrema_sample_count, ir_extrema_reset_count;
extern volatile bool ir_extrema_reset_request, ir_extrema_hold;
extern volatile uint16_t barrel_front_adc_max_5s, barrel_rear_adc_max_5s;
extern volatile uint16_t barrel_front_adc_peak_hold_5s, barrel_rear_adc_peak_hold_5s;
extern volatile uint16_t barrel_front_adc_max_last_5s, barrel_rear_adc_max_last_5s;
extern volatile uint16_t barrel_front_trigger_offset, barrel_rear_trigger_offset;
extern volatile uint32_t ir_peak_sample_count;

void IR_Detection_Init(void);
void IR_Detection_OnPhysicalSample(uint16_t barrel_front_adc,
                                   uint16_t barrel_rear_adc,
                                   uint32_t sample_number);

/* Main-loop event and calibration interfaces; neither touches CAN. */
bool IR_Detection_PeekShotEvent(ShootEvent_t *event);
void IR_Detection_DropShotEvent(void);
uint32_t IR_Detection_GetDroppedEventCount(void);
bool IR_Detection_RequestCalibration(void);
bool IR_Detection_TakeCalibrationResult(uint16_t *old_front,
                                        uint16_t *new_front,
                                        uint16_t *old_rear,
                                        uint16_t *new_rear);
bool IR_Detection_IsCalibrationReady(void);

#endif
