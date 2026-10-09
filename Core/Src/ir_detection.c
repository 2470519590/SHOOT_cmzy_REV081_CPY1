#include "ir_detection.h"
#include "stm32f0xx.h"
#include <string.h>

/* ADC/DMA hands us physical channels in 50 us sample-number units. */
extern volatile uint16_t barrel_front_adc_raw, barrel_rear_adc_raw;
volatile uint16_t barrel_front_baseline, barrel_rear_baseline;
volatile uint16_t barrel_front_trigger_level, barrel_rear_trigger_level;
volatile uint32_t barrel_front_event_count, barrel_rear_event_count;
volatile uint32_t barrel_front_event_sample, barrel_rear_event_sample;
volatile uint16_t barrel_front_peak_adc, barrel_rear_peak_adc;
volatile uint32_t barrel_front_peak_sample, barrel_rear_peak_sample;
volatile uint32_t barrel_front_long_pulse_count, barrel_rear_long_pulse_count;
volatile uint32_t barrel_front_last_pulse_us, barrel_rear_last_pulse_us;
volatile uint32_t ir_shot_count, ir_pair_timeout_count;
volatile uint32_t ir_unpaired_front_count, ir_unpaired_rear_count;
volatile float ir_last_speed_mps;
volatile bool ir_last_speed_valid;
volatile uint32_t ir_last_peak_delta_us;
volatile uint16_t ir_distance_mm = IR_DISTANCE_MM_DEFAULT;
volatile bool barrel_front_active, barrel_rear_active;

volatile uint16_t barrel_front_raw_min = 4095U, barrel_front_raw_max;
volatile uint16_t barrel_rear_raw_min = 4095U, barrel_rear_raw_max;
volatile uint16_t barrel_front_raw_reference, barrel_rear_raw_reference;
volatile uint32_t ir_extrema_sample_count, ir_extrema_reset_count;
volatile bool ir_extrema_reset_request = true, ir_extrema_hold;
volatile uint16_t barrel_front_adc_max_5s, barrel_rear_adc_max_5s;
volatile uint16_t barrel_front_adc_peak_hold_5s, barrel_rear_adc_peak_hold_5s;
volatile uint16_t barrel_front_adc_max_last_5s, barrel_rear_adc_max_last_5s;
volatile uint16_t barrel_front_trigger_offset = IR_FRONT_TRIGGER_OFFSET_DEFAULT;
volatile uint16_t barrel_rear_trigger_offset = IR_REAR_TRIGGER_OFFSET_DEFAULT;
volatile uint32_t ir_peak_sample_count;

/* The candidate queue is for optional speed only, never for shot admission. */
typedef struct {
    uint32_t peak_sample;
    uint16_t peak_adc;
} RearCandidate;

typedef struct {
    bool active;
    uint32_t entry_sample;
    uint32_t peak_sample;
    uint16_t peak_adc;
    uint16_t entry_level;
} PulseState;

static PulseState front_pulse, rear_pulse;
static RearCandidate rear_candidates[IR_REAR_CANDIDATE_CAPACITY];
static uint8_t rear_candidate_head, rear_candidate_count;
static ShootEvent_t shot_events[IR_SHOT_EVENT_CAPACITY];
static uint8_t shot_event_head;
static volatile uint8_t shot_event_count;
static volatile uint32_t shot_event_dropped;
static uint16_t front_histogram[256], rear_histogram[256];
static uint16_t front_bucket_mean_q8[256], rear_bucket_mean_q8[256];
static uint32_t calibration_samples;
static volatile bool calibration_active, baseline_ready, calibration_result_pending;
static bool pending_front_valid;
static uint32_t pending_front_peak_sample;
static uint32_t pending_front_expire_sample;
static uint16_t calibration_old_front, calibration_old_rear;
static uint16_t calibration_new_front, calibration_new_rear;
static int32_t front_baseline_q16, rear_baseline_q16;

static uint16_t histogram_median(const uint16_t *histogram,
                                 const uint16_t *mean_q8, uint32_t total)
{
    uint32_t sum = 0U;
    for (uint16_t value = 0U; value < 256U; ++value) {
        sum += histogram[value];
        if (sum >= (total + 1U) / 2U)
            return (uint16_t)((value << 4) + ((mean_q8[value] + 128U) >> 8));
    }
    return 0U;
}

static void accumulate_baseline(uint16_t sample, uint16_t *histogram,
                                uint16_t *mean_q8)
{
    uint16_t bucket = sample >> 4;
    uint16_t count = ++histogram[bucket];
    int32_t residual_q8 = (int32_t)(sample & 15U) << 8;
    mean_q8[bucket] = (uint16_t)((int32_t)mean_q8[bucket] +
        (residual_q8 - mean_q8[bucket]) / count);
}

static uint16_t trigger_level(uint16_t baseline, uint16_t offset)
{
    uint32_t level = (uint32_t)baseline + offset;
    return level > 4095U ? 4095U : (uint16_t)level;
}

static void reset_pulses_and_candidates(void)
{
    memset(&front_pulse, 0, sizeof(front_pulse));
    memset(&rear_pulse, 0, sizeof(rear_pulse));
    rear_candidate_head = 0U;
    rear_candidate_count = 0U;
    pending_front_valid = false;
    pending_front_peak_sample = 0U;
    pending_front_expire_sample = 0U;
    barrel_front_active = false;
    barrel_rear_active = false;
}

static void finish_calibration(void)
{
    calibration_new_front = histogram_median(front_histogram, front_bucket_mean_q8, calibration_samples);
    calibration_new_rear = histogram_median(rear_histogram, rear_bucket_mean_q8, calibration_samples);
    barrel_front_baseline = calibration_new_front;
    barrel_rear_baseline = calibration_new_rear;
    front_baseline_q16 = (int32_t)barrel_front_baseline << 16;
    rear_baseline_q16 = (int32_t)barrel_rear_baseline << 16;
    barrel_front_trigger_level = trigger_level(barrel_front_baseline, barrel_front_trigger_offset);
    barrel_rear_trigger_level = trigger_level(barrel_rear_baseline, barrel_rear_trigger_offset);
    baseline_ready = true;
    calibration_active = false;
    calibration_result_pending = true;
    reset_pulses_and_candidates();
}

static void add_rear_candidate(uint32_t peak_sample, uint16_t peak_adc)
{
    if (rear_candidate_count == IR_REAR_CANDIDATE_CAPACITY) {
        rear_candidate_head = (uint8_t)((rear_candidate_head + 1U) % IR_REAR_CANDIDATE_CAPACITY);
        rear_candidate_count--;
        ir_unpaired_rear_count++;
    }
    uint8_t index = (uint8_t)((rear_candidate_head + rear_candidate_count) % IR_REAR_CANDIDATE_CAPACITY);
    rear_candidates[index].peak_sample = peak_sample;
    rear_candidates[index].peak_adc = peak_adc;
    rear_candidate_count++;
}

static bool take_best_rear_candidate(uint32_t front_peak_sample, uint32_t *delta_us)
{
    int best = -1;
    uint32_t best_delta = 0xFFFFFFFFUL;
    for (uint8_t i = 0U; i < rear_candidate_count; ++i) {
        uint8_t index = (uint8_t)((rear_candidate_head + i) % IR_REAR_CANDIDATE_CAPACITY);
        uint32_t rear_sample = rear_candidates[index].peak_sample;
        if (rear_sample >= front_peak_sample) continue;
        uint32_t delta_samples = front_peak_sample - rear_sample;
        uint32_t delta = delta_samples * IR_ADC_SAMPLE_US;
        if (delta >= IR_PEAK_DELTA_MIN_US && delta <= IR_PEAK_DELTA_MAX_US && delta < best_delta) {
            best = (int)i;
            best_delta = delta;
        }
    }
    if (best < 0) return false;

    for (uint8_t i = (uint8_t)best; i + 1U < rear_candidate_count; ++i) {
        uint8_t dst = (uint8_t)((rear_candidate_head + i) % IR_REAR_CANDIDATE_CAPACITY);
        uint8_t src = (uint8_t)((rear_candidate_head + i + 1U) % IR_REAR_CANDIDATE_CAPACITY);
        rear_candidates[dst] = rear_candidates[src];
    }
    rear_candidate_count--;
    *delta_us = best_delta;
    return true;
}

static void queue_shot_event_result(uint32_t front_peak_sample,
                                    bool speed_valid,
                                    uint32_t delta_us)
{
    ShootEvent_t event = {0};
    (void)front_peak_sample;
    event.shot_count = ++ir_shot_count;
    event.barrel_mask = 0U;
    event.speed_valid = speed_valid;
    if (event.speed_valid) {
        event.peak_delta_us = delta_us;
        event.speed_mps = ((float)ir_distance_mm * 1000.0f) / (float)delta_us;
        if (event.speed_mps < IR_SPEED_MIN_MPS || event.speed_mps > IR_SPEED_MAX_MPS) {
            event.speed_valid = false;
        }
    }
    if (!event.speed_valid) {
        event.speed_mps = 0.0f;
        event.peak_delta_us = 0U;
        ir_pair_timeout_count++;
        ir_unpaired_front_count++;
    }
    ir_last_speed_mps = event.speed_mps;
    ir_last_speed_valid = event.speed_valid;
    ir_last_peak_delta_us = event.speed_valid ? delta_us : 0U;

    if (shot_event_count == IR_SHOT_EVENT_CAPACITY) {
        shot_event_dropped++;
        return;
    }
    uint8_t index = (uint8_t)((shot_event_head + shot_event_count) % IR_SHOT_EVENT_CAPACITY);
    shot_events[index] = event;
    shot_event_count++;
}

static void queue_shot_event(uint32_t front_peak_sample)
{
    uint32_t delta_us = 0U;
    bool speed_valid = take_best_rear_candidate(front_peak_sample, &delta_us);
    queue_shot_event_result(front_peak_sample, speed_valid, delta_us);
}

static void try_finalize_pending_front(uint32_t sample_number)
{
    uint32_t delta_us = 0U;
    if (!pending_front_valid) return;

    /* A rear candidate may be published several samples after the front
     * pulse exits. Wait for the rear pulse to finish before declaring the
     * speed invalid, but bound the delay so a missing rear sensor cannot
     * hold the shot report forever. */
    if (take_best_rear_candidate(pending_front_peak_sample, &delta_us)) {
        queue_shot_event_result(pending_front_peak_sample, true, delta_us);
        pending_front_valid = false;
        return;
    }
    if ((int32_t)(sample_number - pending_front_expire_sample) >= 0) {
        queue_shot_event_result(pending_front_peak_sample, false, 0U);
        pending_front_valid = false;
    }
}

void IR_Detection_Init(void)
{
    memset(front_histogram, 0, sizeof(front_histogram));
    memset(rear_histogram, 0, sizeof(rear_histogram));
    memset(front_bucket_mean_q8, 0, sizeof(front_bucket_mean_q8));
    memset(rear_bucket_mean_q8, 0, sizeof(rear_bucket_mean_q8));
    memset(shot_events, 0, sizeof(shot_events));
    reset_pulses_and_candidates();
    rear_candidate_head = shot_event_head = 0U;
    rear_candidate_count = shot_event_count = 0U;
    shot_event_dropped = 0U;
    calibration_samples = 0U;
    calibration_active = true;
    baseline_ready = false;
    calibration_result_pending = false;
    ir_last_speed_mps = 0.0f;
    ir_last_speed_valid = false;
    ir_last_peak_delta_us = 0U;
    front_baseline_q16 = 0;
    rear_baseline_q16 = 0;
}

bool IR_Detection_RequestCalibration(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (calibration_active || calibration_result_pending) {
        __set_PRIMASK(primask);
        return false;
    }
    calibration_old_front = barrel_front_baseline;
    calibration_old_rear = barrel_rear_baseline;
    memset(front_histogram, 0, sizeof(front_histogram));
    memset(rear_histogram, 0, sizeof(rear_histogram));
    memset(front_bucket_mean_q8, 0, sizeof(front_bucket_mean_q8));
    memset(rear_bucket_mean_q8, 0, sizeof(rear_bucket_mean_q8));
    calibration_samples = 0U;
    calibration_active = true;
    calibration_result_pending = false;
    reset_pulses_and_candidates();
    __set_PRIMASK(primask);
    return true;
}

void IR_Detection_OnPhysicalSample(uint16_t front, uint16_t rear, uint32_t sample_number)
{
    barrel_front_adc_raw = front;
    barrel_rear_adc_raw = rear;
    static uint32_t window_start, front_peak_time, rear_peak_time;
    if (ir_extrema_reset_request) {
        barrel_front_raw_min = barrel_front_raw_max = barrel_front_raw_reference = front;
        barrel_rear_raw_min = barrel_rear_raw_max = barrel_rear_raw_reference = rear;
        ir_extrema_sample_count = 0U;
        ir_extrema_reset_request = false;
        ir_extrema_reset_count++;
    }
    if (!ir_extrema_hold) {
        if (front < barrel_front_raw_min) barrel_front_raw_min = front;
        if (rear < barrel_rear_raw_min) barrel_rear_raw_min = rear;
        if (front > barrel_front_raw_max) barrel_front_raw_max = front;
        if (rear > barrel_rear_raw_max) barrel_rear_raw_max = rear;
        ir_extrema_sample_count++;
    }
    if ((uint32_t)(sample_number - window_start) >= 100000U) {
        barrel_front_adc_max_last_5s = barrel_front_adc_max_5s;
        barrel_rear_adc_max_last_5s = barrel_rear_adc_max_5s;
        barrel_front_adc_max_5s = barrel_rear_adc_max_5s = 0U;
        window_start = sample_number;
    }
    if (front > barrel_front_adc_max_5s) barrel_front_adc_max_5s = front;
    if (rear > barrel_rear_adc_max_5s) barrel_rear_adc_max_5s = rear;
    if (front >= barrel_front_adc_peak_hold_5s ||
        (uint32_t)(sample_number - front_peak_time) >= 100000U) {
        barrel_front_adc_peak_hold_5s = front;
        front_peak_time = sample_number;
    }
    if (rear >= barrel_rear_adc_peak_hold_5s ||
        (uint32_t)(sample_number - rear_peak_time) >= 100000U) {
        barrel_rear_adc_peak_hold_5s = rear;
        rear_peak_time = sample_number;
    }
    ir_peak_sample_count++;

    if (calibration_active || !baseline_ready) {
        accumulate_baseline(front, front_histogram, front_bucket_mean_q8);
        accumulate_baseline(rear, rear_histogram, rear_bucket_mean_q8);
        calibration_samples++;
        if (calibration_samples >= IR_STARTUP_CALIBRATION_SAMPLES) finish_calibration();
        return;
    }

    barrel_front_trigger_level = trigger_level(barrel_front_baseline, barrel_front_trigger_offset);
    barrel_rear_trigger_level = trigger_level(barrel_rear_baseline, barrel_rear_trigger_offset);

    if (!rear_pulse.active) {
        if (rear >= barrel_rear_trigger_level) {
            rear_pulse.active = true;
            rear_pulse.entry_sample = sample_number;
            rear_pulse.peak_sample = sample_number;
            rear_pulse.peak_adc = rear;
            rear_pulse.entry_level = barrel_rear_trigger_level;
        }
    } else if (rear > rear_pulse.peak_adc) {
        rear_pulse.peak_adc = rear;
        rear_pulse.peak_sample = sample_number;
    } else if (rear < rear_pulse.entry_level) {
        uint32_t exit_sample = sample_number;
        rear_pulse.active = false;
        uint32_t width_us = (exit_sample - rear_pulse.entry_sample) * IR_ADC_SAMPLE_US;
        if (width_us >= IR_REAR_PULSE_MIN_US && width_us <= IR_PULSE_MAX_US) {
            barrel_rear_event_count++;
            barrel_rear_event_sample = rear_pulse.entry_sample;
            barrel_rear_peak_adc = rear_pulse.peak_adc;
            barrel_rear_peak_sample = rear_pulse.peak_sample;
            barrel_rear_last_pulse_us = width_us;
            add_rear_candidate(rear_pulse.peak_sample, rear_pulse.peak_adc);
        } else if (width_us > IR_PULSE_MAX_US) {
            barrel_rear_long_pulse_count++;
        } else {
            ir_unpaired_rear_count++;
        }
    }

    if (!front_pulse.active) {
        if (front >= barrel_front_trigger_level) {
            front_pulse.active = true;
            front_pulse.entry_sample = sample_number;
            front_pulse.peak_sample = sample_number;
            front_pulse.peak_adc = front;
            front_pulse.entry_level = barrel_front_trigger_level;
        }
    } else if (front > front_pulse.peak_adc) {
        front_pulse.peak_adc = front;
        front_pulse.peak_sample = sample_number;
    } else if (front < front_pulse.entry_level) {
        uint32_t exit_sample = sample_number;
        front_pulse.active = false;
        uint32_t width_us = (exit_sample - front_pulse.entry_sample) * IR_ADC_SAMPLE_US;
        if (width_us <= IR_PULSE_MAX_US) {
            barrel_front_event_count++;
            barrel_front_event_sample = front_pulse.entry_sample;
            barrel_front_peak_adc = front_pulse.peak_adc;
            barrel_front_peak_sample = front_pulse.peak_sample;
            barrel_front_last_pulse_us = width_us;
            if (rear_pulse.active) {
                if (pending_front_valid) {
                    /* A new front event cannot overtake an unresolved one in
                     * the supported firing interval. Preserve the older
                     * shot report. */
                    queue_shot_event_result(pending_front_peak_sample, false, 0U);
                }
                pending_front_valid = true;
                pending_front_peak_sample = front_pulse.peak_sample;
                pending_front_expire_sample = sample_number +
                    (IR_PEAK_DELTA_MAX_US + IR_PULSE_MAX_US) / IR_ADC_SAMPLE_US;
            } else {
                /* If rear is idle, its candidate (when present) is already
                 * complete, so preserve the original immediate report path. */
                queue_shot_event(front_pulse.peak_sample);
            }
        } else {
            barrel_front_long_pulse_count++;
        }
    }

    try_finalize_pending_front(sample_number);

    barrel_front_active = front_pulse.active;
    barrel_rear_active = rear_pulse.active;

    /* Check before updating baseline: the current pulse cannot raise its own threshold. */
    front_baseline_q16 += (((int32_t)front << 16) - front_baseline_q16) / IR_BASELINE_TRACK_DIVISOR;
    rear_baseline_q16 += (((int32_t)rear << 16) - rear_baseline_q16) / IR_BASELINE_TRACK_DIVISOR;
    barrel_front_baseline = (uint16_t)(front_baseline_q16 >> 16);
    barrel_rear_baseline = (uint16_t)(rear_baseline_q16 >> 16);
}

bool IR_Detection_PeekShotEvent(ShootEvent_t *event)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool available = event != NULL && shot_event_count != 0U;
    if (available) *event = shot_events[shot_event_head];
    __set_PRIMASK(primask);
    return available;
}

void IR_Detection_DropShotEvent(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (shot_event_count != 0U) {
        shot_event_head = (uint8_t)((shot_event_head + 1U) % IR_SHOT_EVENT_CAPACITY);
        shot_event_count--;
    }
    __set_PRIMASK(primask);
}

uint32_t IR_Detection_GetDroppedEventCount(void)
{
    return shot_event_dropped;
}

bool IR_Detection_TakeCalibrationResult(uint16_t *old_front, uint16_t *new_front,
                                        uint16_t *old_rear, uint16_t *new_rear)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!calibration_result_pending) {
        __set_PRIMASK(primask);
        return false;
    }
    if (old_front != NULL) *old_front = calibration_old_front;
    if (new_front != NULL) *new_front = calibration_new_front;
    if (old_rear != NULL) *old_rear = calibration_old_rear;
    if (new_rear != NULL) *new_rear = calibration_new_rear;
    calibration_result_pending = false;
    __set_PRIMASK(primask);
    return true;
}

bool IR_Detection_IsCalibrationReady(void)
{
    return baseline_ready && !calibration_active;
}
