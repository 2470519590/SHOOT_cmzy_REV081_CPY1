/**
 * @file ws2812_uart.c
 * @brief Two physical LED outputs, encoded at 4 Mbaud with inverted UART TX.
 * Logical pixel 0: RGB_TX3/PB10, onboard GL5050, RGB wire order.
 * Logical pixels 1..8: RGB_TX2/PB6 through the four-pin connector, WS2812 GRB.
 * Only this transport layer maps logical pixels onto physical outputs.
 */
#include "ws2812_uart.h"
#include "main.h"
#include <string.h>

static const uint8_t ws2812_table[4] = {0xEF, 0x8F, 0xEC, 0x8C};
#define LOGICAL_LED_COUNT 9U
#define BYTES_PER_LED 12U
#define RESET_DELAY_LOOPS 4000U

/* Separate buffers remain valid until both independent DMA transfers finish. */
static uint8_t onboard_tx[BYTES_PER_LED];
static uint8_t strip_tx[8U * BYTES_PER_LED];
static bool frame_sent;

static void encode_byte(uint8_t *buffer, uint16_t *pos, uint8_t value)
{
    for (int shift = 6; shift >= 0; shift -= 2) {
        buffer[(*pos)++] = ws2812_table[(value >> shift) & 3U];
    }
}

static void configure_output(USART_TypeDef *uart, DMA_Channel_TypeDef *dma)
{
    dma->CCR &= ~DMA_CCR_EN;
    dma->CPAR = (uint32_t)&uart->TDR;
    dma->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL_0;
    uart->CR3 |= USART_CR3_DMAT;
}

void ws2812_uart_init(void)
{
    frame_sent = false;
    memset(onboard_tx, 0, sizeof(onboard_tx));
    memset(strip_tx, 0, sizeof(strip_tx));
    /* USART3 already uses CH2. USART1 must be remapped away from CH2.
       Both channels use normal DMA, polled completion, no DMA interrupts. */
    __HAL_DMA_REMAP_CHANNEL_ENABLE(DMA_REMAP_USART1_TX_DMA_CH4);
    DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CGIF4;
    configure_output(USART3, DMA1_Channel2);
    configure_output(USART1, DMA1_Channel4);
}

static bool output_busy(USART_TypeDef *uart, DMA_Channel_TypeDef *dma)
{
    if ((dma->CCR & DMA_CCR_EN) != 0U && dma->CNDTR != 0U) return true;
    return (uart->ISR & USART_ISR_TC) == 0U;
}

bool ws2812_uart_busy(void)
{
    return frame_sent && (output_busy(USART3, DMA1_Channel2) ||
                          output_busy(USART1, DMA1_Channel4));
}

static void start_output(USART_TypeDef *uart, DMA_Channel_TypeDef *dma,
                         uint8_t *buffer, uint16_t size)
{
    uart->ICR = USART_ICR_TCCF;
    dma->CMAR = (uint32_t)buffer;
    dma->CNDTR = size;
    dma->CCR |= DMA_CCR_EN;
}

void ws2812_uart_send(const uint32_t *grb, uint8_t count)
{
    if (grb == NULL || count > LOGICAL_LED_COUNT || ws2812_uart_busy()) return;
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1_Channel4->CCR &= ~DMA_CCR_EN;
    DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CGIF4;
    /* Keep both idle-low TX lines low long enough to latch the previous frame. */
    for (volatile uint32_t d = 0; d < RESET_DELAY_LOOPS; d++) {}
    uint16_t onboard_pos = 0U, strip_pos = 0U;
    for (uint8_t i = 0U; i < LOGICAL_LED_COUNT; i++) {
        uint32_t c = i < count ? grb[i] : 0U;
        uint8_t g = (uint8_t)(c >> 16), r = (uint8_t)(c >> 8), b = (uint8_t)c;
        if (i == 0U) {
            encode_byte(onboard_tx, &onboard_pos, r);
            encode_byte(onboard_tx, &onboard_pos, g);
            encode_byte(onboard_tx, &onboard_pos, b);
        } else {
            encode_byte(strip_tx, &strip_pos, g);
            encode_byte(strip_tx, &strip_pos, r);
            encode_byte(strip_tx, &strip_pos, b);
        }
    }
    start_output(USART3, DMA1_Channel2, onboard_tx, sizeof(onboard_tx));
    start_output(USART1, DMA1_Channel4, strip_tx, sizeof(strip_tx));
    frame_sent = true;
}
