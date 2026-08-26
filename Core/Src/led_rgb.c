/**
  ******************************************************************************
  * @file           : led_rgb.c
  * @brief          : WS2812B LED strip control logic
  * @details        : LED[0]  = debug (occlusion → team color, idle → dim yellow)
  *                   LED[1-8] = heat progress bar (referee data → team color)
  *                   Smooth brightness lerp every 100 ms update.
  ******************************************************************************
  */

#include "led_rgb.h"
#include "ws2812_uart.h"

#define HEAT_PER_LED  (256 / LED_HEAT_COUNT)   /* 32 levels per LED            */

/* ========================== Local State ==================================== */

static Team_t    led_team         = TEAM_BLUE;
static bool      led_occluded     = false;
static uint8_t   led_referee_data = 0;
static float     led_desired[LED_COUNT];  /* desired brightness 0.0–1.0        */
static float     led_current[LED_COUNT];  /* current brightness (lerp→desired) */
static bool      shot_effect_active;
static uint32_t  shot_effect_start_tick;

#define SHOT_EFFECT_STEP_MS      10U
#define SHOT_EFFECT_FILL_MS      (LED_HEAT_COUNT * SHOT_EFFECT_STEP_MS)
#define SHOT_EFFECT_OFF_MS       90U
#define SHOT_EFFECT_TOTAL_MS     100U

/* ========================== Helpers ======================================== */

/**
  * @brief  Get team color in GRB format.
  */
static inline uint32_t team_color(void)
{
    return (led_team == TEAM_RED) ? COLOR_PINK : COLOR_LAKE_BLUE;
}

/**
  * @brief  Scale a GRB color by brightness factor (0.0–1.0).
  */
static uint32_t color_dim(uint32_t grb, float factor)
{
    uint8_t g = (uint8_t)(((grb >> 16) & 0xFF) * factor);
    uint8_t r = (uint8_t)(((grb >>  8) & 0xFF) * factor);
    uint8_t b = (uint8_t)(((grb >>  0) & 0xFF) * factor);
    return WS2812_COLOR(g, r, b);
}

/* ========================== Public API ====================================== */

void LedStrip_Init(void)
{
    ws2812_uart_init();

    for (int i = 0; i < LED_COUNT; i++) {
        led_desired[i] = 0.0f;
        led_current[i] = 0.0f;
    }

    /* Self-test: light all 9 LEDs green. */
    uint32_t green[LED_COUNT];
    for (int i = 0; i < LED_COUNT; i++) {
        green[i] = WS2812_COLOR(255, 0, 0);
    }
    ws2812_uart_send(green, LED_COUNT);

    /* Wait for DMA to finish (108 UART bytes @ 3.75M ≈ 288 µs) */
    HAL_Delay(1);
}

void LedStrip_SetTeam(Team_t team)
{
    led_team = team;
}

void LedStrip_SetOcclusion(bool occluded)
{
    led_occluded = occluded;
}

void LedStrip_SetRefereeData(uint8_t data)
{
    led_referee_data = data;
}

void LedStrip_Update(void)
{
    if (ws2812_uart_busy()) return;  /* Previous DMA still in flight          */

    /* ---- Compute desired brightness fractions ---- */
    uint32_t tc = team_color();

    /* LED[0] debug: 100 % team if occluded, 20 % yellow if idle */
    led_desired[LED_DEBUG_IDX] = led_occluded ? 1.0f : 0.20f;

    /* LED[1-8] heat: 256 levels across 8 LEDs (32 per LED) */
    uint8_t full = led_referee_data / HEAT_PER_LED;
    uint8_t frac = led_referee_data % HEAT_PER_LED;
    for (int i = 0; i < LED_HEAT_COUNT; i++) {
        uint8_t idx = LED_HEAT_START + i;
        if (i < full) {
            led_desired[idx] = 1.0f;
        } else if (i == full && frac > 0) {
            led_desired[idx] = (float)frac / (float)HEAT_PER_LED;
        } else {
            led_desired[idx] = 0.0f;
        }
    }

    /* ---- Smooth lerp + send ---- */
    uint32_t out[LED_COUNT];
    for (int i = 0; i < LED_COUNT; i++) {
        float diff = led_desired[i] - led_current[i];
        if (diff > LERP_STEP)       led_current[i] += LERP_STEP;
        else if (diff < -LERP_STEP) led_current[i] -= LERP_STEP;
        else                        led_current[i]  = led_desired[i];

        if (led_current[i] <= 0.0f) {
            out[i] = 0;
        } else if (i == LED_DEBUG_IDX && !led_occluded) {
            out[i] = color_dim(COLOR_YELLOW_DIM, led_current[i] / 0.20f);
        } else {
            out[i] = color_dim(tc, led_current[i]);
        }
    }
    ws2812_uart_send(out, LED_COUNT);
}

void LedStrip_ShowFaultAlert(uint32_t tick_ms)
{
    uint32_t out[LED_COUNT];
    uint32_t color;

    if (ws2812_uart_busy()) return;

    /* Three colours per 500 ms gives one red/blue/green cycle every 500 ms. */
    switch ((tick_ms / 167U) % 3U) {
    case 0U: color = WS2812_COLOR(0, 255, 0); break;   /* red */
    case 1U: color = WS2812_COLOR(0, 0, 255); break;   /* blue */
    default: color = WS2812_COLOR(255, 0, 0); break;   /* green */
    }
    for (int i = 0; i < LED_COUNT; i++) out[i] = color;
    ws2812_uart_send(out, LED_COUNT);
}

/**
  * @brief  Start the valid-shot rear-to-front sweep on LEDs 8 down to 1.
  * @note   The effect is displayed only by the automatic LED layer; faults
  *         and explicit debug/CAN LED commands keep their existing priority.
  */
void LedStrip_StartShotEffect(uint32_t tick_ms)
{
    shot_effect_start_tick = tick_ms;
    shot_effect_active = true;
}

/**
  * @brief  Render a 100 ms valid-shot effect. LEDs fill front-to-rear in
  *         10 ms steps, then all eight heat LEDs turn off at 90 ms.
  * @return true while the automatic LED layer must remain overridden.
  */
bool LedStrip_ProcessShotEffect(uint32_t tick_ms)
{
    if (!shot_effect_active) return false;

    uint32_t elapsed = tick_ms - shot_effect_start_tick;
    if (elapsed >= SHOT_EFFECT_TOTAL_MS) {
        if (ws2812_uart_busy()) return true;
        shot_effect_active = false;
        LedStrip_Update();  /* immediately restore the ordinary LED state */
        return false;
    }

    if (ws2812_uart_busy()) return true;

    uint32_t out[LED_COUNT] = {0};
    /* Keep the upper status LED blue while the shot sweep is running. */
    out[LED_DEBUG_IDX] = COLOR_LAKE_BLUE;
    if (elapsed < SHOT_EFFECT_OFF_MS) {
        uint8_t lit = (uint8_t)(elapsed / SHOT_EFFECT_STEP_MS) + 1U;
        if (lit > LED_HEAT_COUNT) lit = LED_HEAT_COUNT;

        /* Physical direction is LED[1] toward LED[8]. */
        for (uint8_t i = 0; i < lit; i++) {
            out[LED_HEAT_START + i] =
                WS2812_COLOR(255, 0, 0);  /* green, same colour as boot test */
        }
    }
    ws2812_uart_send(out, LED_COUNT);
    return true;
}

/* ========================== Debug / CAN Override =========================== */

/**
  * @brief  Manual test pattern — write g_led_cmd from debugger.
  *         pattern: 1=green, 2=red, 3=blue, 4=team, 5=off
  *         count:   0 = all 9 LEDs, else limit
  */
void LedStrip_TestPattern(uint8_t pattern, uint8_t count)
{
    if (ws2812_uart_busy()) return;

    uint8_t n = (count == 0) ? LED_COUNT : (count > LED_COUNT ? LED_COUNT : count);
    uint32_t color = 0;

    switch (pattern) {
    case 1:  color = WS2812_COLOR(255,   0,   0); break; /* green  */
    case 2:  color = WS2812_COLOR(  0, 255,   0); break; /* red    */
    case 3:  color = WS2812_COLOR(  0,   0, 255); break; /* blue   */
    case 4:  color = (led_team == TEAM_RED) ? COLOR_PINK : COLOR_LAKE_BLUE; break;
    default: color = 0; break;  /* 5 or other = off */
    }

    uint32_t out[LED_COUNT];
    for (int i = 0; i < LED_COUNT; i++) {
        out[i] = (i < n) ? color : 0;
    }
    ws2812_uart_send(out, LED_COUNT);
}

/**
  * @brief  Apply a CAN LED command (team / heat / test).
  */
void LedStrip_ApplyCommand(const LedCommand_t *cmd)
{
    uint32_t out[LED_COUNT];
    uint32_t tc;
    uint8_t  full, frac;

    switch (cmd->cmd) {
    case CAN_LED_TEAM_RED:
        LedStrip_SetTeam(TEAM_RED);
        break;
    case CAN_LED_TEAM_BLUE:
        LedStrip_SetTeam(TEAM_BLUE);
        break;
    case CAN_LED_HEAT_DATA:
        /* Bypass lerp — direct write with fractional edge brightness */
        LedStrip_SetRefereeData(cmd->heat_data);
        LedStrip_SetOcclusion(true);
        tc   = (led_team == TEAM_RED) ? COLOR_PINK : COLOR_LAKE_BLUE;
        full = cmd->heat_data / HEAT_PER_LED;
        frac = cmd->heat_data % HEAT_PER_LED;
        if (ws2812_uart_busy()) break;
        out[LED_DEBUG_IDX] = tc;
        for (int i = 0; i < LED_HEAT_COUNT; i++) {
            if (i < full)
                out[LED_HEAT_START + i] = tc;
            else if (i == full && frac > 0)
                out[LED_HEAT_START + i] = color_dim(tc,
                    (float)frac / (float)HEAT_PER_LED);
            else
                out[LED_HEAT_START + i] = 0;
        }
        ws2812_uart_send(out, LED_COUNT);
        break;
    case CAN_LED_TEST_GREEN:
        LedStrip_TestPattern(1, 0);
        break;
    case CAN_LED_TEST_RED:
        LedStrip_TestPattern(2, 0);
        break;
    case CAN_LED_TEST_BLUE:
        LedStrip_TestPattern(3, 0);
        break;
    case CAN_LED_TEST_OFF:
        LedStrip_TestPattern(5, 0);
        break;
    default:
        break;
    }
}
