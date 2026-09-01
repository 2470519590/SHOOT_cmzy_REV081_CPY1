/**
  ******************************************************************************
  * @file           : led_rgb.h
  * @brief          : WS2812B LED strip control — projectile status indication
  * @description    : 9 LEDs in series:
  *                   [0]     On-board debug LED
  *                   [1-8]   Shooting heat progress bar (8-bit referee data)
  *                   Red  team → pink,  Blue team → lake blue
  *                   Occluded → 100 % team color on LED[0]
  *                   Idle     →  20 % yellow    on LED[0]
  *                   Updated every 100 ms via TIM15 tick.
  ******************************************************************************
  */
#ifndef __LED_RGB_H
#define __LED_RGB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include <stdbool.h>
#include "can_protocol.h"

/* ========================== Constants ====================================== */

#define LED_COUNT             9
#define LED_DEBUG_IDX         0
#define LED_HEAT_START        1
#define LED_HEAT_COUNT        8

/* Team colors in GRB format (WS2812_COLOR macro in ws2812_uart.h) */
#define COLOR_PINK            WS2812_COLOR( 50, 255, 100)   /* R=255 G=50  B=100 */
#define COLOR_LAKE_BLUE       WS2812_COLOR(180,   0, 255)   /* R=0   G=180 B=255 */
#define COLOR_YELLOW_DIM      WS2812_COLOR( 40,  51,   0)   /* R=255 G=200 B=0 ×0.2 */

/* Brightness lerp step per 100 ms update (0.0–1.0) */
#define LERP_STEP             0.30f

/* ========================== Types ========================================== */

typedef enum {
    TEAM_RED  = 0,
    TEAM_BLUE = 1,
} Team_t;

/* ========================== Public API ===================================== */

void LedStrip_Init(void);
void LedStrip_SetTeam(Team_t team);
void LedStrip_SetOverheatAlert(bool active);
void LedStrip_SetRefereeData(uint8_t data);
void LedStrip_Update(void);    /* Call every 100 ms (TIM15 tick)              */
void LedStrip_ShowFaultAlert(uint32_t tick_ms); /* red/blue/green cyclic alert */
void LedStrip_StartShotEffect(uint32_t tick_ms);
bool LedStrip_ProcessShotEffect(uint32_t tick_ms);

/* Debug / CAN test patterns */
void LedStrip_TestPattern(uint8_t pattern, uint8_t count);  /* count=0→all 9 LEDs */
void LedStrip_ApplyCommand(const LedCommand_t *cmd);         /* CAN command        */

#ifdef __cplusplus
}
#endif

#endif /* __LED_RGB_H */
