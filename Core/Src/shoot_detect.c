/**
  ******************************************************************************
  * @file           : shoot_detect.c
  * @brief          : Multi-projectile barrel tracking via FIFO bitmask
  * @details        : Up to BARREL_SLOTS projectiles tracked simultaneously.
  *                   Rear trigger allocates a slot (sets bit → 1),
  *                   front trigger drains the oldest slot (clears bit → 0).
  *                   Per-slot timeout prevents stalled slots.
  *                   TIM16 @ 20 kHz provides 50 µs tick resolution.
  ******************************************************************************
  */

#include "shoot_detect.h"
#include "vcnl4040.h"
#include <string.h>

/* TIM16 is configured as 20 kHz free-running counter in main.c
   (48 MHz / 2400 = 20 kHz, 50 µs/tick, wraps every 3.28 s)          */
#define SPEED_TIMER             TIM16
#define SPEED_TICKS_TO_MPS(t)   (1000.0f / (float)(t))

/* ========================== Static Helpers ================================== */

static inline uint16_t speed_timer_ticks(void)
{
    return (uint16_t)(SPEED_TIMER->CNT);
}

/**
  * @brief  Find index of first 0 bit in mask (returns 0..4, or -1 if full).
  */
static int barrel_find_free(uint8_t mask)
{
    for (int i = 0; i < BARREL_SLOTS; i++) {
        if (!(mask & (1 << i))) return i;
    }
    return -1;
}

/**
  * @brief  Find index of first 1 bit in mask (FIFO: oldest entry).
  */
static int barrel_find_oldest(uint8_t mask)
{
    for (int i = 0; i < BARREL_SLOTS; i++) {
        if (mask & (1 << i)) return i;
    }
    return -1;
}

/* ========================== Public API ====================================== */

void ShootDetect_Init(ShootDetect_t *det,
                      I2C_HandleTypeDef *front_i2c,
                      I2C_HandleTypeDef *rear_i2c)
{
    memset(det, 0, sizeof(ShootDetect_t));
    det->front_i2c = front_i2c;
    det->rear_i2c  = rear_i2c;

    det->speed_min_mps       = DEFAULT_SPEED_MIN_MPS;
    det->speed_max_mps       = DEFAULT_SPEED_MAX_MPS;
    det->timeout_ms          = DEFAULT_TIMEOUT_MS;
    det->threshold_away_offset = THRESHOLD_AWAY_OFFSET;
    det->threshold_last_update_ms = 0U;
}

void ShootDetect_SetParams(ShootDetect_t *det,
                           float min_mps, float max_mps,
                           uint32_t timeout_ms)
{
    det->speed_min_mps = min_mps;
    det->speed_max_mps = max_mps;
    det->timeout_ms    = timeout_ms;
}

bool ShootDetect_Calibrate(ShootDetect_t *det)
{
    uint32_t sum_front = 0, sum_rear = 0;
    uint16_t sample;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        sample = VCNL4040_GetProximity(det->front_i2c);
        if (sample == 0xFFFF) return false;
        sum_front += sample;

        sample = VCNL4040_GetProximity(det->rear_i2c);
        if (sample == 0xFFFF) return false;
        sum_rear += sample;

        HAL_Delay(2);
    }

    det->front_baseline = (uint16_t)(sum_front / CALIBRATION_SAMPLES);
    det->rear_baseline  = (uint16_t)(sum_rear  / CALIBRATION_SAMPLES);

    /* A projectile makes PS_DATA fall.  Use the VCNL4040 AWAY mode, which
       asserts INT when PS_DATA drops below PS_THDL.  Keep PS_THDH at the
       calibrated baseline although AWAY mode does not use it. */
    uint16_t f_low = (det->front_baseline > det->threshold_away_offset) ?
                     (uint16_t)(det->front_baseline - det->threshold_away_offset) : 0U;
    uint16_t r_low = (det->rear_baseline > det->threshold_away_offset) ?
                     (uint16_t)(det->rear_baseline - det->threshold_away_offset) : 0U;

    VCNL4040_SetProximityLowThreshold(det->front_i2c, f_low);
    VCNL4040_SetProximityHighThreshold(det->front_i2c, det->front_baseline);
    VCNL4040_SetProximityLowThreshold(det->rear_i2c,  r_low);
    VCNL4040_SetProximityHighThreshold(det->rear_i2c,  det->rear_baseline);

    det->front_threshold_low = f_low;
    det->rear_threshold_low  = r_low;

    VCNL4040_EnableProximityInterrupt(det->front_i2c, VCNL4040_PS_INT_AWAY);
    VCNL4040_EnableProximityInterrupt(det->rear_i2c,  VCNL4040_PS_INT_AWAY);
    VCNL4040_GetInterruptStatus(det->front_i2c);
    VCNL4040_GetInterruptStatus(det->rear_i2c);

    return true;
}

/* ---- ISR callbacks ------------------------------------------------------- */

void ShootDetect_RearTrigger(ShootDetect_t *det)
{
    uint16_t now = speed_timer_ticks();

    det->rear_int_count++;
    det->rear_int_triggered = true;

    int slot = barrel_find_free(det->barrel_mask);
    if (slot < 0) return;   /* barrel full — discard */

    det->barrel_mask      |= (uint8_t)(1 << slot);
    det->rear_tick[slot]   = now;
    det->slot_timeout[slot] = HAL_GetTick();
}

void ShootDetect_FrontTrigger(ShootDetect_t *det)
{
    uint16_t now = speed_timer_ticks();

    det->front_int_count++;
    det->front_int_triggered = true;

    int slot = barrel_find_oldest(det->barrel_mask);
    if (slot < 0) return;   /* no projectile in barrel — spurious front event */

    det->timer_wrapped = (now < det->rear_tick[slot]);

    uint16_t delta = now - det->rear_tick[slot];   /* unsigned handles wrap */
    if (delta == 0) {
        det->barrel_mask &= (uint8_t)~(1 << slot);
        return;
    }

    float speed = SPEED_TICKS_TO_MPS(delta);
    if (speed >= det->speed_min_mps && speed <= det->speed_max_mps) {
        det->last_speed_mps = speed;
        det->shot_count++;

        /* Queue the confirmed event. CAN transmission is deferred to main. */
        uint8_t next = (uint8_t)((det->event_queue_head + 1U) % SHOOT_EVENT_QUEUE_SIZE);
        if (next != det->event_queue_tail) {
            ShootEvent_t *event = &det->event_queue[det->event_queue_head];
            event->shot_count = det->shot_count;
            event->speed_mps = speed;
            event->barrel_mask = (uint8_t)(det->barrel_mask & BARREL_MASK);
            det->event_queue_head = next;
        } else {
            det->event_queue_dropped++;
        }
    }

    det->barrel_mask &= (uint8_t)~(1 << slot);
}

/* ---- Periodic processing (main loop @ 1 kHz) ----------------------------- */

void ShootDetect_Process(ShootDetect_t *det)
{
    /* Clear VCNL4040 interrupt latches */
    if (det->front_int_triggered) {
        VCNL4040_GetInterruptStatus(det->front_i2c);
        det->front_int_triggered = false;
    }
    if (det->rear_int_triggered) {
        VCNL4040_GetInterruptStatus(det->rear_i2c);
        det->rear_int_triggered = false;
    }

    /* Per-slot timeout: any projectile stuck > timeout_ms gets evicted */
    uint32_t now_ms = HAL_GetTick();
    for (int i = 0; i < BARREL_SLOTS; i++) {
        if (det->barrel_mask & (1 << i)) {
            if ((now_ms - det->slot_timeout[i]) >= det->timeout_ms) {
                det->barrel_mask &= (uint8_t)~(1 << i);
            }
        }
    }
}

/* ---- Getters ------------------------------------------------------------- */

uint32_t ShootDetect_GetCount(const ShootDetect_t *det)
{
    return det->shot_count;
}

uint32_t ShootDetect_GetFrontIntCount(const ShootDetect_t *det)
{
    return det->front_int_count;
}

uint32_t ShootDetect_GetRearIntCount(const ShootDetect_t *det)
{
    return det->rear_int_count;
}

float ShootDetect_GetLastSpeed(const ShootDetect_t *det)
{
    return det->last_speed_mps;
}

uint8_t ShootDetect_GetState(const ShootDetect_t *det)
{
    return det->barrel_mask;   /* now exposes which slots are occupied         */
}

/**
  * @brief  Slowly follow the PS background and move the AWAY threshold.
  *         There is intentionally no event freeze or change-size lockout:
  *         the baseline must recover from a genuine environment step.
  */
void ShootDetect_UpdateAdaptiveThresholds(ShootDetect_t *det,
                                           uint16_t front_ps,
                                           uint16_t rear_ps,
                                           uint32_t now_ms)
{
    int32_t diff;

    if (front_ps != 0xFFFFU) {
        diff = (int32_t)front_ps - (int32_t)det->front_baseline;
        int32_t step = diff / (1 << ADAPTIVE_BASELINE_SHIFT);
        /* Integer EMA would otherwise stop with a 1..15 count steady-state
           error. Force a one-count final step so the baseline converges. */
        if (step == 0 && diff != 0) step = (diff > 0) ? 1 : -1;
        det->front_baseline = (uint16_t)((int32_t)det->front_baseline + step);
    }
    if (rear_ps != 0xFFFFU) {
        diff = (int32_t)rear_ps - (int32_t)det->rear_baseline;
        int32_t step = diff / (1 << ADAPTIVE_BASELINE_SHIFT);
        if (step == 0 && diff != 0) step = (diff > 0) ? 1 : -1;
        det->rear_baseline = (uint16_t)((int32_t)det->rear_baseline + step);
    }

    if ((uint32_t)(now_ms - det->threshold_last_update_ms) <
        ADAPTIVE_THRESHOLD_PERIOD_MS) {
        return;
    }
    det->threshold_last_update_ms = now_ms;

    uint16_t front_low = (det->front_baseline > det->threshold_away_offset) ?
        (uint16_t)(det->front_baseline - det->threshold_away_offset) : 0U;
    uint16_t rear_low = (det->rear_baseline > det->threshold_away_offset) ?
        (uint16_t)(det->rear_baseline - det->threshold_away_offset) : 0U;

    if (front_low != det->front_threshold_low) {
        VCNL4040_SetProximityLowThreshold(det->front_i2c, front_low);
        VCNL4040_SetProximityHighThreshold(det->front_i2c, det->front_baseline);
        det->front_threshold_low = front_low;
    }
    if (rear_low != det->rear_threshold_low) {
        VCNL4040_SetProximityLowThreshold(det->rear_i2c, rear_low);
        VCNL4040_SetProximityHighThreshold(det->rear_i2c, det->rear_baseline);
        det->rear_threshold_low = rear_low;
    }
}

bool ShootDetect_PeekEvent(const ShootDetect_t *det, ShootEvent_t *event)
{
    if (det->event_queue_tail == det->event_queue_head) return false;

    *event = det->event_queue[det->event_queue_tail];
    return true;
}

void ShootDetect_DropEvent(ShootDetect_t *det)
{
    if (det->event_queue_tail != det->event_queue_head) {
        det->event_queue_tail = (uint8_t)((det->event_queue_tail + 1U) % SHOOT_EVENT_QUEUE_SIZE);
    }
}

uint32_t ShootDetect_GetDroppedEventCount(const ShootDetect_t *det)
{
    return det->event_queue_dropped;
}
