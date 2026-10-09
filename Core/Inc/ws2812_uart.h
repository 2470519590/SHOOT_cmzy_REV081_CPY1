/**
  ******************************************************************************
  * @file           : ws2812_uart.h
  * @brief          : Board LED and external WS2812 strip via two UART DMA outputs
  * @description    : Each color byte becomes 4 UART bytes, encoding 2 LED
  *                   bits per UART frame. Both UARTs run at 4 Mbaud.
  *                   Pixel 0: RGB_TX3/PB10 (USART3, DMA CH2, RGB).
  *                   Pixels 1..8: RGB_TX2/PB6 (USART1, DMA CH4, GRB).
  *                   TX inversion makes idle low for the WS2812 reset pulse.
  ******************************************************************************
  */
#ifndef __WS2812_UART_H
#define __WS2812_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include <stdbool.h>

/* Logical colours are GRB. The transport alone handles the board GL5050 RGB
   order and external WS2812 GRB order, plus routing to physical connectors. */
#define WS2812_COLOR(g, r, b)   (((uint32_t)(g) << 16) | ((uint32_t)(r) << 8) | (uint32_t)(b))

void ws2812_uart_init(void);
bool ws2812_uart_busy(void);
void ws2812_uart_send(const uint32_t *grb, uint8_t count);

#ifdef __cplusplus
}
#endif

#endif /* __WS2812_UART_H */
