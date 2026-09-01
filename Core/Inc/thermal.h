#ifndef __THERMAL_H
#define __THERMAL_H

#include <stdint.h>
#include <stdbool.h>

#define THERMAL_HEAT_LIMIT       45U  /* competition heat capacity */
#define THERMAL_COOL_PERIOD_MS   500U  /* one heat point per 500 ms = 2/s */
#define THERMAL_OVERHEAT_LED_MS  3000U

void Thermal_Init(uint32_t now_ms);
void Thermal_Update(uint32_t now_ms);
uint8_t Thermal_AddShot(uint32_t now_ms);
bool Thermal_SetHeat(uint8_t heat, uint32_t now_ms);
uint8_t Thermal_GetHeat(void);
bool Thermal_IsOverheatIndicatorActive(uint32_t now_ms);

#endif /* __THERMAL_H */
