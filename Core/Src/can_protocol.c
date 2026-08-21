/**
  ******************************************************************************
  * @file           : can_protocol.c
  * @brief          : CAN referee-system slave — Rx commands, Tx status
  ******************************************************************************
  */

#include "can_protocol.h"

static CAN_HandleTypeDef  *can_handle = NULL;
static ShootData_Report_t  shoot_data = {0};
static LedCommand_t        led_cmd    = { .source = LED_SRC_NORMAL,
                                          .cmd = 0, .heat_data = 0, .team = 1 };
extern volatile uint8_t g_heat_debug;

/* ---- LED command accessor ----------------------------------------------- */
const LedCommand_t *CANProtocol_GetLedCommand(void)
{
    return &led_cmd;
}

/* ---- Init --------------------------------------------------------------- */
void CANProtocol_Init(CAN_HandleTypeDef *hcan)
{
    can_handle = hcan;
}

/* ---- Shoot data refresh ------------------------------------------------- */
void CANProtocol_UpdateData(const ShootData_Report_t *data)
{
    if (data != NULL) shoot_data = *data;
}

/* ---- Send shoot status response (0x300) --------------------------------- */
static void send_shoot_report(CAN_HandleTypeDef *hcan)
{
    CAN_TxHeaderTypeDef  tx_header;
    CAN_ShootReport_t    payload;
    uint32_t             tx_mailbox;

    payload.shot_count      = shoot_data.shot_count;
    payload.last_speed_cmps = (uint16_t)(shoot_data.last_speed_mps * 100.0f
                                         + 0.5f);
    payload.barrel_mask     = shoot_data.barrel_mask;
    payload.heat_level      = shoot_data.heat_level;

    tx_header.StdId              = CAN_BOARD_RESPONSE_ID;
    tx_header.ExtId              = 0;
    tx_header.IDE                = CAN_ID_STD;
    tx_header.RTR                = CAN_RTR_DATA;
    tx_header.DLC                = sizeof(CAN_ShootReport_t);
    tx_header.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(hcan, &tx_header, (uint8_t *)&payload, &tx_mailbox);
}

/* ---- LED command handler ------------------------------------------------ */
static void handle_led_cmd(const CAN_LedCmd_t *rx)
{
    switch (rx->cmd) {
    case CAN_LED_TEAM_RED:
        led_cmd.source = LED_SRC_DEBUG;
        led_cmd.cmd    = CAN_LED_TEAM_RED;
        led_cmd.team   = 0;
        break;
    case CAN_LED_TEAM_BLUE:
        led_cmd.source = LED_SRC_DEBUG;
        led_cmd.cmd    = CAN_LED_TEAM_BLUE;
        led_cmd.team   = 1;
        break;
    case CAN_LED_HEAT_DATA:
        led_cmd.source    = LED_SRC_DEBUG;
        led_cmd.cmd       = CAN_LED_HEAT_DATA;
        led_cmd.heat_data = rx->heat_data;
        g_heat_debug      = rx->heat_data;
        break;
    case CAN_LED_TEST_GREEN:
    case CAN_LED_TEST_RED:
    case CAN_LED_TEST_BLUE:
    case CAN_LED_TEST_OFF:
        led_cmd.source = LED_SRC_DEBUG;
        led_cmd.cmd    = rx->cmd;
        break;
    case CAN_LED_NORMAL:
    default:
        led_cmd.source = LED_SRC_NORMAL;
        led_cmd.cmd    = 0;
        break;
    }
}

/* ---- Rx callback -------------------------------------------------------- */
void CANProtocol_RxCallback(CAN_HandleTypeDef *hcan, uint32_t RxFifo)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t             rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, RxFifo, &rx_header, rx_data) != HAL_OK)
        return;

    /* Shoot data query → respond */
    if (rx_header.StdId == CAN_REFEREE_QUERY_ID &&
        rx_header.IDE   == CAN_ID_STD &&
        rx_header.RTR   == CAN_RTR_DATA) {
        send_shoot_report(hcan);
        return;
    }

    /* LED command */
    if (rx_header.StdId == CAN_LED_CMD_ID &&
        rx_header.IDE   == CAN_ID_STD &&
        rx_header.RTR   == CAN_RTR_DATA) {
        handle_led_cmd((const CAN_LedCmd_t *)rx_data);
        return;
    }
}
