/**
  ******************************************************************************
  * @file           : ws2812_uart.h
  * @brief          : WS2812B LED driver via USART3 + TX inversion + DMA
  * @description    : Each color byte becomes 4 UART bytes, encoding 2 LED
  *                   bits per UART frame. USART3 runs at 4 Mbaud on PB10.
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

/* The LED driver stores colors as GRB for legacy callers; the GL5050 device
   receives them on the wire as RGB (the encoder performs that conversion). */
#define WS2812_COLOR(g, r, b)   (((uint32_t)(g) << 16) | ((uint32_t)(r) << 8) | (uint32_t)(b))

void ws2812_uart_init(void);
bool ws2812_uart_busy(void);
void ws2812_uart_send(const uint32_t *grb, uint8_t count);

#ifdef __cplusplus
}
#endif

#endif /* __WS2812_UART_H */
