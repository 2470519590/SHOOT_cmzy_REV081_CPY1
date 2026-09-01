#include "thermal.h"
#include "stm32f0xx_hal.h"

static volatile uint8_t thermal_heat;
static volatile uint32_t thermal_cool_tick;
static volatile uint32_t thermal_overheat_led_until;

void Thermal_Init(uint32_t now_ms)
{
    thermal_heat = 0U;
    thermal_cool_tick = now_ms;
    thermal_overheat_led_until = 0U;
}

void Thermal_Update(uint32_t now_ms)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    while (thermal_heat > 0U &&
           (uint32_t)(now_ms - thermal_cool_tick) >= THERMAL_COOL_PERIOD_MS) {
        thermal_heat--;
        thermal_cool_tick += THERMAL_COOL_PERIOD_MS;
    }
    if (thermal_heat == 0U) {
        thermal_cool_tick = now_ms;
    }
    __set_PRIMASK(primask);
}

uint8_t Thermal_AddShot(uint32_t now_ms)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (thermal_heat < THERMAL_HEAT_LIMIT) {
        thermal_heat++;
        if (thermal_heat == THERMAL_HEAT_LIMIT) {
            thermal_overheat_led_until = now_ms + THERMAL_OVERHEAT_LED_MS;
        }
    }
    uint8_t result = thermal_heat;
    __set_PRIMASK(primask);
    return result;
}

bool Thermal_SetHeat(uint8_t heat, uint32_t now_ms)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (heat > THERMAL_HEAT_LIMIT) {
        __set_PRIMASK(primask);
        return false;
    }
    thermal_heat = heat;
    thermal_cool_tick = now_ms;
    if (heat == THERMAL_HEAT_LIMIT) {
        thermal_overheat_led_until = now_ms + THERMAL_OVERHEAT_LED_MS;
    }
    __set_PRIMASK(primask);
    return true;
}

uint8_t Thermal_GetHeat(void)
{
    return thermal_heat;
}

bool Thermal_IsOverheatIndicatorActive(uint32_t now_ms)
{
    return (int32_t)(thermal_overheat_led_until - now_ms) > 0;
}
