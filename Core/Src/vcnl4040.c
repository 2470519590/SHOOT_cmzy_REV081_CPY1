/**
  ******************************************************************************
  * @file           : vcnl4040.c
  * @brief          : VCNL4040 proximity sensor driver, STM32 HAL I2C backend
  ******************************************************************************
  */

#include "vcnl4040.h"

static HAL_StatusTypeDef VCNL4040_ReadReg(I2C_HandleTypeDef *hi2c,
                                          uint8_t reg, uint16_t *data)
{
    uint8_t buf[2] = {0};
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(hi2c, VCNL4040_I2C_ADDR, reg,
                              I2C_MEMADD_SIZE_8BIT, buf, 2, 100);
    if (status == HAL_OK) {
        *data = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    }
    return status;
}

static HAL_StatusTypeDef VCNL4040_WriteReg(I2C_HandleTypeDef *hi2c,
                                           uint8_t reg, uint16_t data)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(data & 0xFF);
    buf[1] = (uint8_t)((data >> 8) & 0xFF);

    return HAL_I2C_Mem_Write(hi2c, VCNL4040_I2C_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, buf, 2, 100);
}

static void VCNL4040_ModifyReg(I2C_HandleTypeDef *hi2c, uint8_t reg,
                               uint16_t mask, uint16_t value)
{
    uint16_t reg_val = 0;

    if (VCNL4040_ReadReg(hi2c, reg, &reg_val) == HAL_OK) {
        reg_val = (reg_val & ~mask) | (value & mask);
        VCNL4040_WriteReg(hi2c, reg, reg_val);
    }
}

HAL_StatusTypeDef VCNL4040_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    uint16_t dev_id = 0;

    status = HAL_I2C_IsDeviceReady(hi2c, VCNL4040_I2C_ADDR, 2, 10);
    if (status != HAL_OK) {
        return status;
    }

    status = VCNL4040_ReadReg(hi2c, VCNL4040_REG_DEVICE_ID, &dev_id);
    if (status != HAL_OK) {
        return status;
    }
    if (dev_id != VCNL4040_EXPECTED_ID) {
        return HAL_ERROR;
    }

    /* PS_CONF1_2: PS enabled, 16-bit proximity, interrupt disabled for now. */
    uint16_t ps_conf = 0x0800
                     | ((uint16_t)(VCNL4040_DEFAULT_PROX_INT_TIME & 0x07) << 1);
    status = VCNL4040_WriteReg(hi2c, VCNL4040_REG_PS_CONF1_2, ps_conf);
    if (status != HAL_OK) return status;

    /* PS_MS: LED current is in bits [10:8]; WHITE_EN bit[15] stays clear. */
    uint16_t ps_ms = ((uint16_t)(VCNL4040_DEFAULT_LED_CURRENT & 0x07) << 8);
    status = VCNL4040_WriteReg(hi2c, VCNL4040_REG_PS_MS, ps_ms);
    if (status != HAL_OK) return status;

    /* ALS is not used. */
    status = VCNL4040_WriteReg(hi2c, VCNL4040_REG_ALS_CONF, 0x0001);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

bool VCNL4040_CheckDevice(I2C_HandleTypeDef *hi2c)
{
    uint16_t dev_id = 0;

    if (VCNL4040_ReadReg(hi2c, VCNL4040_REG_DEVICE_ID, &dev_id) != HAL_OK) {
        return false;
    }

    return (dev_id == VCNL4040_EXPECTED_ID);
}

void VCNL4040_EnableProximity(I2C_HandleTypeDef *hi2c, bool enable)
{
    VCNL4040_ModifyReg(hi2c, VCNL4040_REG_PS_CONF1_2, 0x0001,
                       enable ? 0x0000 : 0x0001);
}

void VCNL4040_EnableAmbientLight(I2C_HandleTypeDef *hi2c, bool enable)
{
    VCNL4040_ModifyReg(hi2c, VCNL4040_REG_ALS_CONF, 0x0001,
                       enable ? 0x0000 : 0x0001);
}

void VCNL4040_EnableWhiteLight(I2C_HandleTypeDef *hi2c, bool enable)
{
    VCNL4040_ModifyReg(hi2c, VCNL4040_REG_PS_MS, 0x8000,
                       enable ? 0x8000 : 0x0000);
}

void VCNL4040_SetLEDCurrent(I2C_HandleTypeDef *hi2c, uint8_t current_code)
{
    if (current_code > 7) current_code = 7;
    VCNL4040_ModifyReg(hi2c, VCNL4040_REG_PS_MS, 0x0700,
                       (uint16_t)(current_code << 8));
}

void VCNL4040_SetLEDDutyCycle(I2C_HandleTypeDef *hi2c, uint8_t duty_code)
{
    if (duty_code > 3) duty_code = 3;
    VCNL4040_ModifyReg(hi2c, VCNL4040_REG_PS_CONF1_2, 0x00C0,
                       (uint16_t)(duty_code << 6));
}

void VCNL4040_SetProximityIntegrationTime(I2C_HandleTypeDef *hi2c,
                                          uint8_t it_code)
{
    if (it_code > 7) it_code = 7;
    VCNL4040_ModifyReg(hi2c, VCNL4040_REG_PS_CONF1_2, 0x000E,
                       (uint16_t)(it_code << 1));
}

void VCNL4040_SetProximityHighResolution(I2C_HandleTypeDef *hi2c, bool enable)
{
    VCNL4040_ModifyReg(hi2c, VCNL4040_REG_PS_CONF1_2, 0x0800,
                       enable ? 0x0800 : 0x0000);
}

uint16_t VCNL4040_GetProximity(I2C_HandleTypeDef *hi2c)
{
    uint16_t data = 0xFFFF;

    if (VCNL4040_ReadReg(hi2c, VCNL4040_REG_PS_DATA, &data) != HAL_OK) {
        return 0xFFFF;
    }

    return data;
}

void VCNL4040_SetProximityLowThreshold(I2C_HandleTypeDef *hi2c,
                                       uint16_t threshold)
{
    VCNL4040_WriteReg(hi2c, VCNL4040_REG_PS_THDL, threshold);
}

void VCNL4040_SetProximityHighThreshold(I2C_HandleTypeDef *hi2c,
                                        uint16_t threshold)
{
    VCNL4040_WriteReg(hi2c, VCNL4040_REG_PS_THDH, threshold);
}

void VCNL4040_EnableProximityInterrupt(I2C_HandleTypeDef *hi2c, uint8_t mode)
{
    if (mode > 3) mode = VCNL4040_PS_INT_DISABLE;
    VCNL4040_ModifyReg(hi2c, VCNL4040_REG_PS_CONF1_2, 0x0300,
                       (uint16_t)(mode << 8));
}

uint8_t VCNL4040_GetInterruptStatus(I2C_HandleTypeDef *hi2c)
{
    uint16_t int_flag = 0;

    if (VCNL4040_ReadReg(hi2c, VCNL4040_REG_INT_FLAG, &int_flag) != HAL_OK) {
        return 0;
    }

    return (uint8_t)(int_flag & 0xFF);
}
