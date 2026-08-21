/**
  ******************************************************************************
  * @file           : shoot_detect.h
  * @brief          : Dual-sensor projectile detection & speed measurement
  * @description    : Two VCNL4040 sensors spaced 50 mm apart along the barrel.
  *                   Up to 5 projectiles tracked simultaneously via bitmask
  *                   FIFO: rear trigger fills a slot, front trigger drains the
  *                   oldest slot.  Per-slot timeout prevents stall.
  *                   TIM16 @ 20 kHz provides 50 µs tick resolution.
  ******************************************************************************
  */
#ifndef __SHOOT_DETECT_H
#define __SHOOT_DETECT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include <stdbool.h>

/* ========================== Fixed Parameters ================================ */

#define SENSOR_SPACING_M        0.050f        /* 50 mm between sensors          */
#define CALIBRATION_SAMPLES     20            /* Number of baseline readings    */
#define THRESHOLD_HIGH_OFFSET   500           /* close: baseline + 500           */
#define THRESHOLD_LOW_OFFSET    100           /* away:  baseline + 100 (< HIGH) */
#define DEFAULT_TIMEOUT_MS      50            /* Per-slot timeout (ms)           */
#define DEFAULT_SPEED_MIN_MPS   0.0f          /* Minimum valid speed   (m/s)    */
#define DEFAULT_SPEED_MAX_MPS   50.0f         /* Maximum valid speed   (m/s)    */

#define BARREL_SLOTS            5             /* Max simultaneous projectiles    */
#define BARREL_MASK             ((1 << BARREL_SLOTS) - 1)  /* 0x1F              */

/* ========================== Type Definitions ================================ */

typedef struct {
    /* ---- Multi-projectile FIFO barrel mask ---- */
    volatile uint8_t  barrel_mask;        /* bits[4:0]: 1 = in barrel            */
    volatile uint16_t rear_tick [BARREL_SLOTS]; /* TIM16 CNT on rear trigger     */
    volatile uint32_t slot_timeout[BARREL_SLOTS]; /* HAL_GetTick on slot fill    */

    /* ---- Counters & results ---- */
    uint32_t      shot_count;
    uint32_t      front_int_count;        /* Raw EXTI count (every rising edge)  */
    uint32_t      rear_int_count;         /* Raw EXTI count (every rising edge)  */
    float         last_speed_mps;

    /* ---- Interrupt pending flags ---- */
    volatile bool front_int_triggered;
    volatile bool rear_int_triggered;

    /* ---- Config ---- */
    uint32_t      timeout_ms;             /* Per-slot timeout window             */
    float         speed_min_mps;
    float         speed_max_mps;

    /* ---- Calibration ---- */
    uint16_t      front_baseline;
    uint16_t      rear_baseline;
    uint16_t      threshold_high_offset;  /* INT LOW  trigger margin             */
    uint16_t      threshold_low_offset;   /* INT HIGH release margin             */

    /* ---- I2C handles ---- */
    I2C_HandleTypeDef *front_i2c;         /* hi2c1                               */
    I2C_HandleTypeDef *rear_i2c;          /* hi2c2                               */

    /* ---- Diagnostic ---- */
    volatile bool     timer_wrapped;      /* counter wrapped between triggers     */
} ShootDetect_t;

/* ========================== Public API ====================================== */

void ShootDetect_Init(ShootDetect_t *det,
                      I2C_HandleTypeDef *front_i2c,
                      I2C_HandleTypeDef *rear_i2c);
void ShootDetect_SetParams(ShootDetect_t *det,
                           float min_mps, float max_mps,
                           uint32_t timeout_ms);
bool ShootDetect_Calibrate(ShootDetect_t *det);

/* ISR callbacks — keep fast */
void ShootDetect_RearTrigger(ShootDetect_t *det);
void ShootDetect_FrontTrigger(ShootDetect_t *det);

/* Periodic processing — called every 1 ms from main loop */
void ShootDetect_Process(ShootDetect_t *det);

/* Result access */
uint32_t ShootDetect_GetCount(const ShootDetect_t *det);
uint32_t ShootDetect_GetFrontIntCount(const ShootDetect_t *det);
uint32_t ShootDetect_GetRearIntCount(const ShootDetect_t *det);
float    ShootDetect_GetLastSpeed(const ShootDetect_t *det);
uint8_t  ShootDetect_GetState(const ShootDetect_t *det);  /* returns barrel_mask */

#ifdef __cplusplus
}
#endif

#endif /* __SHOOT_DETECT_H */
