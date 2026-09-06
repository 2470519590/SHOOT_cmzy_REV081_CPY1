#ifndef __THERMAL_H
#define __THERMAL_H

#include <stdint.h>
#include <stdbool.h>

#define THERMAL_HEAT_LIMIT       200U /* competition heat capacity */
#define THERMAL_HEAT_PER_SHOT    10U  /* heat added by one valid shot */
#define THERMAL_COOL_PER_SECOND  10U  /* heat removed each elapsed second */
#define THERMAL_COOL_PERIOD_MS   1000U
#define THERMAL_OVERHEAT_LED_MS  3000U

void Thermal_Init(uint32_t now_ms);
void Thermal_Update(uint32_t now_ms);
uint8_t Thermal_AddShot(uint32_t now_ms);
bool Thermal_SetHeat(uint8_t heat, uint32_t now_ms);
uint8_t Thermal_GetHeat(void);
bool Thermal_IsOverheatIndicatorActive(uint32_t now_ms);

#endif /* __THERMAL_H */
