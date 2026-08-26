/**
  ******************************************************************************
  * @file           : can_protocol.c
  * @brief          : CAN referee-system slave — Rx commands, Tx status
  ******************************************************************************
  */

#include "can_protocol.h"

extern CAN_ErrorStats_t g_can_stats;

static CAN_HandleTypeDef  *can_handle = NULL;
static ShootData_Report_t  shoot_data = {0};
static LedCommand_t        led_cmd    = { .source = LED_SRC_NORMAL,
                                          .cmd = 0, .heat_data = 0, .team = 1 };
static volatile bool calibration_requested;
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

static HAL_StatusTypeDef send_single_byte(uint16_t id, uint8_t value)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox = 0;

    if (can_handle == NULL || HAL_CAN_GetState(can_handle) != HAL_CAN_STATE_LISTENING) {
        return HAL_ERROR;
    }
    tx_header.StdId = id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 1;
    tx_header.TransmitGlobalTime = DISABLE;
    return HAL_CAN_AddTxMessage(can_handle, &tx_header, &value, &tx_mailbox);
}

HAL_StatusTypeDef CANProtocol_SendWeakFault(uint8_t weak_mask)
{
    return send_single_byte(CAN_WEAK_FAULT_ID, weak_mask);
}

HAL_StatusTypeDef CANProtocol_SendStrongFault(uint8_t strong_mask)
{
    return send_single_byte(CAN_STRONG_FAULT_ID, strong_mask);
}

HAL_StatusTypeDef CANProtocol_SendBoot(void)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint8_t unused = 0U;
    uint32_t tx_mailbox = 0;

    if (can_handle == NULL || HAL_CAN_GetState(can_handle) != HAL_CAN_STATE_LISTENING) {
        return HAL_ERROR;
    }
    tx_header.StdId = CAN_BOOT_ID;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 0;
    tx_header.TransmitGlobalTime = DISABLE;
    return HAL_CAN_AddTxMessage(can_handle, &tx_header, &unused, &tx_mailbox);
}

HAL_StatusTypeDef CANProtocol_SendCalibrationAck(uint8_t status)
{
    return send_single_byte(CAN_CALIBRATE_ACK_ID, status);
}

bool CANProtocol_TakeCalibrationRequest(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool requested = calibration_requested;
    calibration_requested = false;
    __set_PRIMASK(primask);
    return requested;
}

/* ---- Periodic heartbeat: ID 0x200, one standard data frame per second ---- */
HAL_StatusTypeDef CANProtocol_SendHeartbeat(void)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint8_t payload[8] = {0};
    uint32_t tx_mailbox = 0;
    uint32_t uptime_s = HAL_GetTick() / 1000U;
    HAL_StatusTypeDef status;

    if (can_handle == NULL || HAL_CAN_GetState(can_handle) != HAL_CAN_STATE_LISTENING) {
        g_can_stats.tx_heartbeat_fail++;
        return HAL_ERROR;
    }

    payload[0] = 0xA5;
    payload[1] = 0x01;
    payload[2] = (uint8_t)(uptime_s >> 0);
    payload[3] = (uint8_t)(uptime_s >> 8);
    payload[4] = (uint8_t)(uptime_s >> 16);
    payload[5] = (uint8_t)(uptime_s >> 24);
    payload[6] = g_heat_debug;
    payload[7] = (uint8_t)(shoot_data.barrel_mask & 0x1FU);

    tx_header.StdId = CAN_HEARTBEAT_ID;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    status = HAL_CAN_AddTxMessage(can_handle, &tx_header, payload, &tx_mailbox);
    if (status == HAL_OK) {
        g_can_stats.tx_heartbeat_ok++;
    } else {
        g_can_stats.tx_heartbeat_fail++;
    }
    return status;
}

/* ---- Send one valid-shot event (0x230) ------------------------------- */
HAL_StatusTypeDef CANProtocol_SendShotEvent(const ShootEvent_t *event)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint8_t payload[8] = {0};
    uint32_t tx_mailbox = 0;
    uint16_t speed_cmps;

    if (event == NULL || can_handle == NULL ||
        HAL_CAN_GetState(can_handle) != HAL_CAN_STATE_LISTENING) {
        return HAL_ERROR;
    }

    speed_cmps = (uint16_t)(event->speed_mps * 100.0f + 0.5f);
    payload[0] = (uint8_t)(event->shot_count >> 0);
    payload[1] = (uint8_t)(event->shot_count >> 8);
    payload[2] = (uint8_t)(event->shot_count >> 16);
    payload[3] = (uint8_t)(event->shot_count >> 24);
    payload[4] = (uint8_t)(speed_cmps >> 0);
    payload[5] = (uint8_t)(speed_cmps >> 8);
    payload[6] = (uint8_t)(event->barrel_mask & 0x1FU);
    payload[7] = 0x01U; /* valid-shot event */

    tx_header.StdId = CAN_SHOT_EVENT_ID;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(can_handle, &tx_header, payload, &tx_mailbox);
}

/* ---- Send shoot status response (0x232) --------------------------------- */
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

    if (HAL_CAN_AddTxMessage(hcan, &tx_header, (uint8_t *)&payload,
                             &tx_mailbox) == HAL_OK) {
        g_can_stats.tx_status_ok++;
    } else {
        g_can_stats.tx_status_fail++;
    }
}

/* ---- LED command handler ------------------------------------------------ */
static bool handle_led_cmd(const CAN_LedCmd_t *rx)
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
        led_cmd.source = LED_SRC_NORMAL;
        led_cmd.cmd    = 0;
        break;
    default:
        return false;
    }
    return true;
}

/* ---- Acknowledge one accepted LED command (0x234) ---------------------- */
static void send_led_ack(CAN_HandleTypeDef *hcan, const uint8_t *payload)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox = 0;

    tx_header.StdId = CAN_LED_ACK_ID;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(hcan, &tx_header, (uint8_t *)payload,
                             &tx_mailbox) == HAL_OK) {
        g_can_stats.tx_led_ack_ok++;
    } else {
        g_can_stats.tx_led_ack_fail++;
    }
}

/* ---- Rx callback -------------------------------------------------------- */
void CANProtocol_RxCallback(CAN_HandleTypeDef *hcan, uint32_t RxFifo)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t             rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, RxFifo, &rx_header, rx_data) != HAL_OK)
        return;

    g_can_stats.rx_frames++;

    /* Calibration takes roughly 40 ms of I2C reads and delays. This ISR only
       records the request; the main loop performs it and sends the ACK. */
    if (rx_header.StdId == CAN_CALIBRATE_REQUEST_ID &&
        rx_header.IDE   == CAN_ID_STD &&
        rx_header.RTR   == CAN_RTR_DATA &&
        rx_header.DLC   == 0U) {
        calibration_requested = true;
        g_can_stats.rx_calibrate_request++;
        return;
    }

    /* Shoot data query → respond */
    if (rx_header.StdId == CAN_REFEREE_QUERY_ID &&
        rx_header.IDE   == CAN_ID_STD &&
        rx_header.RTR   == CAN_RTR_DATA &&
        (rx_header.DLC == 0U || rx_header.DLC == 8U)) {
        send_shoot_report(hcan);
        g_can_stats.rx_query_0x231++;
        return;
    }

    /* LED command */
    if (rx_header.StdId == CAN_LED_CMD_ID &&
        rx_header.IDE   == CAN_ID_STD &&
        rx_header.RTR   == CAN_RTR_DATA &&
        rx_header.DLC   == 8U) {
        if (!handle_led_cmd((const CAN_LedCmd_t *)rx_data)) {
            g_can_stats.rx_unknown++;
            return;
        }
        g_can_stats.rx_led_cmd++;
        send_led_ack(hcan, rx_data);
        return;
    }

    g_can_stats.rx_unknown++;
}
