/**
  ******************************************************************************
  * @file           : ws2812_uart.c
  * @brief          : WS2812B via USART3 @ 3.75 Mbaud + TX inversion + DMA
  * @details        : One normal DMA transfer sends exactly one LED frame.
  *                   TX inversion makes the UART idle state a WS2812 reset low.
  ******************************************************************************
  */

#include "ws2812_uart.h"
#include "main.h"
#include <string.h>

static const uint8_t ws2812_table[4] = { 0xEF, 0x8F, 0xEC, 0x8C };

#define LED_COUNT          9
#define BYTES_PER_LED     12    /* G(4) + R(4) + B(4) */
#define TX_BUF_SIZE        (LED_COUNT * BYTES_PER_LED)
#define RESET_DELAY_LOOPS  4000U /* >50 us at 48 MHz, conservative for latch */

static uint8_t tx_buf[TX_BUF_SIZE];
static bool frame_sent = false;

extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart3;

static void encode_color_byte(uint8_t value, uint16_t *pos)
{
    tx_buf[(*pos)++] = ws2812_table[(value >> 6) & 0x03];
    tx_buf[(*pos)++] = ws2812_table[(value >> 4) & 0x03];
    tx_buf[(*pos)++] = ws2812_table[(value >> 2) & 0x03];
    tx_buf[(*pos)++] = ws2812_table[(value >> 0) & 0x03];
}

static void ws2812_reset_delay(void)
{
    for (volatile uint32_t d = 0; d < RESET_DELAY_LOOPS; d++) {}
}

void ws2812_uart_init(void)
{
    memset(tx_buf, 0, TX_BUF_SIZE);
    frame_sent = false;

    /* USART3 (3M, 8N1, TXINV) and DMA1_CH2 already configured by
       MX_USART3_UART_Init + MX_DMA_Init.  Only enable DMA TX request.     */
    USART3->CR3 |= USART_CR3_DMAT;
    DMA1->IFCR = DMA_IFCR_CGIF2;
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1_Channel2->CPAR  = (uint32_t)(&USART3->TDR);
    DMA1_Channel2->CCR   = DMA_CCR_MINC | DMA_CCR_DIR;
}

bool ws2812_uart_busy(void)
{
    if (!frame_sent) {
        return false;
    }

    if (((DMA1_Channel2->CCR & DMA_CCR_EN) != 0U) && (DMA1_Channel2->CNDTR != 0U)) {
        return true;
    }

    return (USART3->ISR & USART_ISR_TC) == 0U;
}

void ws2812_uart_send(const uint32_t *grb, uint8_t count)
{
    if ((grb == NULL) || (count > LED_COUNT) || ws2812_uart_busy()) {
        return;
    }

    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1->IFCR = DMA_IFCR_CGIF2;
    USART3->ICR = USART_ICR_TCCF;

    if (frame_sent) {
        ws2812_reset_delay();
    }

    memset(tx_buf, 0, TX_BUF_SIZE);

    /* WS2812B data order is GRB. */
    uint16_t pos = 0;
    for (uint8_t i = 0; i < count; i++) {
        uint32_t c = grb[i];
        uint8_t g = (uint8_t)(c >> 16);
        uint8_t r = (uint8_t)(c >> 8);
        uint8_t b = (uint8_t)(c);

        encode_color_byte(g, &pos);
        encode_color_byte(r, &pos);
        encode_color_byte(b, &pos);
    }

    DMA1_Channel2->CNDTR = TX_BUF_SIZE;
    DMA1_Channel2->CMAR  = (uint32_t)tx_buf;
    DMA1_Channel2->CPAR  = (uint32_t)(&USART3->TDR);
    DMA1_Channel2->CCR  |= DMA_CCR_EN;
    frame_sent = true;
}
