/**
  ******************************************************************************
  * @file           : vcnl4040.h
  * @brief          : VCNL4040 proximity/ambient light sensor driver header
  * @description    : Hardware I2C driver for Vishay VCNL4040 sensor on STM32 HAL.
  *                   Provides register-level access, configuration, and
  *                   interrupt-based proximity detection setup.
  * @reference      : Adafruit_VCNL4040 library (Arduino) — register map & config
  ******************************************************************************
  */
#ifndef __VCNL4040_H
#define __VCNL4040_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include <stdbool.h>

/* ========================== I2C Address ==================================== */
#define VCNL4040_I2C_ADDR       (0x60 << 1)   /* 7-bit addr shifted for HAL    */

/* ========================== Register Map (8-bit command code) ============== */
#define VCNL4040_REG_ALS_CONF    0x00  /* Ambient light configuration         */
#define VCNL4040_REG_ALS_THDH    0x01  /* ALS high threshold (16-bit)         */
#define VCNL4040_REG_ALS_THDL    0x02  /* ALS low threshold  (16-bit)         */
#define VCNL4040_REG_PS_CONF1_2  0x03  /* Proximity sensor config 1+2 (16-bit)*/
#define VCNL4040_REG_PS_MS       0x04  /* PS measurement config  (16-bit)     */
#define VCNL4040_REG_PS_THDL     0x06  /* PS low threshold       (16-bit)     */
#define VCNL4040_REG_PS_THDH     0x07  /* PS high threshold      (16-bit)     */
#define VCNL4040_REG_PS_DATA     0x08  /* PS data output         (16-bit)     */
#define VCNL4040_REG_ALS_DATA    0x09  /* ALS data output        (16-bit)     */
#define VCNL4040_REG_WHITE_DATA  0x0A  /* White light data       (16-bit)     */
#define VCNL4040_REG_INT_FLAG    0x0B  /* Interrupt status       (16-bit)     */
#define VCNL4040_REG_DEVICE_ID   0x0C  /* Device ID (expected 0x0186)         */

#define VCNL4040_EXPECTED_ID     0x0186
/* A normal two-byte register transfer completes in under 1 ms at 100 kHz.
   Never allow an I2C fault to stall projectile interrupt servicing for 100 ms. */
#define VCNL4040_I2C_TIMEOUT_MS  5U

/* ========================== PS_CONF1_2 bit fields ========================== */
/* [0]     PS_SD       : Proximity sensor shutdown (1=disable)                */
/* [3:1]   PS_IT       : Proximity integration time                           */
/* [5:3]   PS_PERS     : Persistence (interrupt number of measurements)       */
/* [7:6]   PS_DUTY      : LED duty cycle                                      */
/* [9:8]   PS_INT       : Interrupt type                                      */
/* [10]    PS_SMART_PERS: Smart persistence                                   */
/* [11]    PS_HD        : High-resolution (0=12bit, 1=16bit)                  */
/* [15:12] Reserved                                                           */

/* Proximity interrupt type selection (PS_CONF1_2[9:8]) */
#define VCNL4040_PS_INT_DISABLE      0x00
#define VCNL4040_PS_INT_CLOSE        0x01
#define VCNL4040_PS_INT_AWAY         0x02
#define VCNL4040_PS_INT_CLOSE_AWAY   0x03

/* ========================== PS_MS bit fields =============================== */
/* [10:8]  PS_LED_I   : LED current selection                                 */
/* [15]    PS_WHITE_EN: White light enable                                    */
/* Others reserved                                                           */

/* ========================== INT_FLAG bits (lower byte) ====================== */
#define VCNL4040_INT_PROX_AWAY       (1 << 0)
#define VCNL4040_INT_PROX_CLOSE      (1 << 1)
#define VCNL4040_INT_ALS_HIGH        (1 << 4)
#define VCNL4040_INT_ALS_LOW         (1 << 5)

/* ========================== Default configuration =========================== */
#define VCNL4040_DEFAULT_LED_CURRENT      0   /* 0~7 → 50~200mA, 4≈140mA      */
#define VCNL4040_DEFAULT_PROX_INT_TIME    1   /* 0~7 → 1T~8T, 3=2T (~1ms @1T) */
#define VCNL4040_DEFAULT_PS_DUTY          0   /* 0 → 1/40, highest duty / fastest response */
#define VCNL4040_DEFAULT_HIGH_RES         true /* 16-bit proximity data        */

/* ========================== Public API ====================================== */

/**
  * @brief  Initialise VCNL4040 with default settings for projectile detection.
  * @param  hi2c : Pointer to I2C handle (I2C1 = front, I2C2 = rear).
  * @retval HAL_StatusTypeDef (HAL_OK on success).
  */
HAL_StatusTypeDef VCNL4040_Init(I2C_HandleTypeDef *hi2c);

/**
  * @brief  Read Device ID register and compare with expected value 0x0186.
  * @param  hi2c : Pointer to I2C handle.
  * @retval true if device responds with correct ID, false otherwise.
  */
bool VCNL4040_CheckDevice(I2C_HandleTypeDef *hi2c);

/* ---- Enable / disable individual sensor channels ---- */
void VCNL4040_EnableProximity(I2C_HandleTypeDef *hi2c, bool enable);
void VCNL4040_EnableAmbientLight(I2C_HandleTypeDef *hi2c, bool enable);
void VCNL4040_EnableWhiteLight(I2C_HandleTypeDef *hi2c, bool enable);

/* ---- Proximity sensor tuning ---- */
void VCNL4040_SetLEDCurrent(I2C_HandleTypeDef *hi2c, uint8_t current_code);
void VCNL4040_SetLEDDutyCycle(I2C_HandleTypeDef *hi2c, uint8_t duty_code);
void VCNL4040_SetProximityIntegrationTime(I2C_HandleTypeDef *hi2c,
                                          uint8_t it_code);
void VCNL4040_SetProximityHighResolution(I2C_HandleTypeDef *hi2c, bool enable);

/* ---- Data reading ---- */
uint16_t VCNL4040_GetProximity(I2C_HandleTypeDef *hi2c);

/* ---- Threshold (used for interrupt-based detection) ---- */
void VCNL4040_SetProximityLowThreshold(I2C_HandleTypeDef *hi2c,
                                       uint16_t threshold);
void VCNL4040_SetProximityHighThreshold(I2C_HandleTypeDef *hi2c,
                                        uint16_t threshold);

/* ---- Interrupt configuration ---- */
void VCNL4040_EnableProximityInterrupt(I2C_HandleTypeDef *hi2c, uint8_t mode);

/**
  * @brief  Read and return the INT_FLAG register (also clears interrupt).
  * @param  hi2c : Pointer to I2C handle.
  * @retval 8-bit interrupt status; test with VCNL4040_INT_xxx masks.
  */
uint8_t VCNL4040_GetInterruptStatus(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif /* __VCNL4040_H */
