/**
  ******************************************************************************
  * @file           : can_protocol.c
  * @brief          : CAN referee-system slave — Rx commands, Tx status
  ******************************************************************************
  */

#include "can_protocol.h"
#include "thermal.h"
#include <string.h>

extern CAN_ErrorStats_t g_can_stats;

static CAN_HandleTypeDef  *can_handle = NULL;
static ShootData_Report_t  shoot_data = {0};
static LedCommand_t        led_cmd    = { .source = LED_SRC_NORMAL,
                                           .cmd = 0, .heat_data = 0, .team = 1 };
static volatile bool calibration_requested;
static volatile uint8_t shoot_report_request_count;
#define LED_ACK_QUEUE_DEPTH 4U
static uint8_t led_ack_queue[LED_ACK_QUEUE_DEPTH][8];
static volatile uint8_t led_ack_head;
static volatile uint8_t led_ack_tail;
static volatile uint32_t led_ack_drop_count;

#define CAN_PROTOCOL_TX_TIMEOUT_MS 2U

/* ---- LED command accessor ----------------------------------------------- */
const LedCommand_t *CANProtocol_GetLedCommand(void)
{
    return &led_cmd;
}

/* ---- Init --------------------------------------------------------------- */
void CANProtocol_Init(CAN_HandleTypeDef *hcan)
{
    can_handle = hcan;
    shoot_report_request_count = 0U;
    led_ack_head = 0U;
    led_ack_tail = 0U;
    led_ack_drop_count = 0U;
}

/* ---- Shoot data refresh ------------------------------------------------- */
void CANProtocol_UpdateData(const ShootData_Report_t *data)
{
    if (data != NULL) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        shoot_data = *data;
        __set_PRIMASK(primask);
    }
}

static HAL_StatusTypeDef send_single_byte(uint16_t id, uint8_t value)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox = 0;

    if (can_handle == NULL ||
        HAL_CAN_GetState(can_handle) != HAL_CAN_STATE_LISTENING) {
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

    if (can_handle == NULL ||
        HAL_CAN_GetState(can_handle) != HAL_CAN_STATE_LISTENING) {
        return HAL_ERROR;
    }
    tx_header.StdId = CAN_BOOT_ID;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 0;
    tx_header.TransmitGlobalTime = DISABLE;
    return HAL_CAN_AddTxMessage(can_handle, &tx_header, &unused, &tx_mailbox);
}

HAL_StatusTypeDef CANProtocol_SendCalibrationAck(uint16_t old_front,
                                                 uint16_t new_front,
                                                 uint16_t old_rear,
                                                 uint16_t new_rear)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint8_t payload[8];
    uint32_t tx_mailbox = 0;

    if (can_handle == NULL ||
        HAL_CAN_GetState(can_handle) != HAL_CAN_STATE_LISTENING) {
        return HAL_ERROR;
    }

    payload[0] = (uint8_t)(old_front >> 0);
    payload[1] = (uint8_t)(old_front >> 8);
    payload[2] = (uint8_t)(new_front >> 0);
    payload[3] = (uint8_t)(new_front >> 8);
    payload[4] = (uint8_t)(old_rear >> 0);
    payload[5] = (uint8_t)(old_rear >> 8);
    payload[6] = (uint8_t)(new_rear >> 0);
    payload[7] = (uint8_t)(new_rear >> 8);
    tx_header.StdId = CAN_CALIBRATE_ACK_ID;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;
    return HAL_CAN_AddTxMessage(can_handle, &tx_header, payload, &tx_mailbox);
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

    if (can_handle == NULL ||
        HAL_CAN_GetState(can_handle) != HAL_CAN_STATE_LISTENING) {
        g_can_stats.tx_heartbeat_fail++;
        return HAL_ERROR;
    }

    payload[0] = 0xA5;
    payload[1] = 0x01;
    payload[2] = (uint8_t)(uptime_s >> 0);
    payload[3] = (uint8_t)(uptime_s >> 8);
    payload[4] = (uint8_t)(uptime_s >> 16);
    payload[5] = (uint8_t)(uptime_s >> 24);
    payload[6] = shoot_data.heat_level;
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

/* HAL_CAN_AddTxMessage only proves that a
 * mailbox accepted the frame, not that it won arbitration and received a CAN
 * ACK. The shot queue may be popped only after TXOK. */
static HAL_StatusTypeDef send_frame_confirmed(const CAN_TxHeaderTypeDef *header,
                                              uint8_t *payload)
{
    uint32_t mailbox = 0U;
    uint32_t rqcp;
    uint32_t txok;
    uint32_t started;

    if (can_handle == NULL || header == NULL ||
        HAL_CAN_GetState(can_handle) != HAL_CAN_STATE_LISTENING ||
        (can_handle->Instance->ESR & CAN_ESR_BOFF) != 0U) {
        return HAL_ERROR;
    }
    if (HAL_CAN_AddTxMessage(can_handle, (CAN_TxHeaderTypeDef *)header,
                             payload, &mailbox) != HAL_OK) {
        return HAL_ERROR;
    }
    if (mailbox == CAN_TX_MAILBOX0) {
        rqcp = CAN_TSR_RQCP0; txok = CAN_TSR_TXOK0;
    } else if (mailbox == CAN_TX_MAILBOX1) {
        rqcp = CAN_TSR_RQCP1; txok = CAN_TSR_TXOK1;
    } else if (mailbox == CAN_TX_MAILBOX2) {
        rqcp = CAN_TSR_RQCP2; txok = CAN_TSR_TXOK2;
    } else {
        return HAL_ERROR;
    }
    started = HAL_GetTick();
    while ((can_handle->Instance->TSR & rqcp) == 0U) {
        if ((can_handle->Instance->ESR & CAN_ESR_BOFF) != 0U ||
            (uint32_t)(HAL_GetTick() - started) >= CAN_PROTOCOL_TX_TIMEOUT_MS) {
            (void)HAL_CAN_AbortTxRequest(can_handle, mailbox);
            return HAL_TIMEOUT;
        }
    }
    if ((can_handle->Instance->TSR & txok) == 0U) {
        can_handle->Instance->TSR = rqcp;
        return HAL_ERROR;
    }
    can_handle->Instance->TSR = rqcp;
    return HAL_OK;
}

/* ---- Send one valid-shot event (0x230) ------------------------------- */
HAL_StatusTypeDef CANProtocol_SendShotEvent(const ShootEvent_t *event)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint8_t payload[8] = {0};
    uint16_t speed_deci_mps;

    if (event == NULL || can_handle == NULL ||
        HAL_CAN_GetState(can_handle) != HAL_CAN_STATE_LISTENING) {
        g_can_stats.tx_shot_event_fail++;
        return HAL_ERROR;
    }

    speed_deci_mps = event->speed_valid
        ? (uint16_t)(event->speed_mps * 10.0f + 0.5f)
        : 0xFFFFU;
    payload[0] = (uint8_t)(event->shot_count >> 0);
    payload[1] = (uint8_t)(event->shot_count >> 8);
    payload[2] = (uint8_t)(event->shot_count >> 16);
    payload[3] = (uint8_t)(event->shot_count >> 24);
    payload[4] = (uint8_t)(speed_deci_mps >> 0);
    payload[5] = (uint8_t)(speed_deci_mps >> 8);
    payload[6] = 0U;
    payload[7] = event->heat_level;

    tx_header.StdId = CAN_SHOT_EVENT_ID;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    HAL_StatusTypeDef status = send_frame_confirmed(&tx_header, payload);
    if (status == HAL_OK) {
        g_can_stats.tx_shot_event_ok++;
    } else {
        g_can_stats.tx_shot_event_fail++;
    }
    return status;
}

/* ---- Send shoot status response (0x232) --------------------------------- */
static HAL_StatusTypeDef send_shoot_report(void)
{
    CAN_TxHeaderTypeDef  tx_header;
    CAN_ShootReport_t    payload;
    uint32_t             primask;

    /* Snapshot all fields together; this function is called from CAN IRQ while
       the main loop refreshes shoot_data periodically. */
    primask = __get_PRIMASK();
    __disable_irq();
    payload.shot_count      = shoot_data.shot_count;
    payload.last_speed_deci_mps = (shoot_data.last_speed_mps > 0.0f)
        ? (uint16_t)(shoot_data.last_speed_mps * 10.0f + 0.5f)
        : 0xFFFFU;
    payload.barrel_mask     = 0U;
    payload.heat_level      = shoot_data.heat_level;
    __set_PRIMASK(primask);

    tx_header.StdId              = CAN_BOARD_RESPONSE_ID;
    tx_header.ExtId              = 0;
    tx_header.IDE                = CAN_ID_STD;
    tx_header.RTR                = CAN_RTR_DATA;
    tx_header.DLC                = sizeof(CAN_ShootReport_t);
    tx_header.TransmitGlobalTime = DISABLE;

    HAL_StatusTypeDef status = send_frame_confirmed(&tx_header, (uint8_t *)&payload);
    if (status == HAL_OK) {
        g_can_stats.tx_status_ok++;
    } else {
        g_can_stats.tx_status_fail++;
    }
    return status;
}

void CANProtocol_Task(void)
{
    uint32_t primask;
    uint8_t have_request;

    primask = __get_PRIMASK();
    __disable_irq();
    have_request = shoot_report_request_count;
    __set_PRIMASK(primask);
    if (have_request != 0U && send_shoot_report() == HAL_OK) {
        primask = __get_PRIMASK();
        __disable_irq();
        if (shoot_report_request_count != 0U) {
            shoot_report_request_count--;
        }
        __set_PRIMASK(primask);
    }

    if (led_ack_tail != led_ack_head) {
        CAN_TxHeaderTypeDef tx_header = {0};
        uint32_t tx_mailbox = 0U;
        tx_header.StdId = CAN_LED_ACK_ID;
        tx_header.IDE = CAN_ID_STD;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = 8U;
        tx_header.TransmitGlobalTime = DISABLE;
        if (can_handle != NULL &&
            HAL_CAN_AddTxMessage(can_handle, &tx_header,
                                 led_ack_queue[led_ack_tail],
                                 &tx_mailbox) == HAL_OK) {
            g_can_stats.tx_led_ack_ok++;
            primask = __get_PRIMASK();
            __disable_irq();
            led_ack_tail = (uint8_t)((led_ack_tail + 1U) % LED_ACK_QUEUE_DEPTH);
            __set_PRIMASK(primask);
        } else {
            g_can_stats.tx_led_ack_fail++;
        }
    }
}

/* ---- LED command handler ------------------------------------------------ */
static bool handle_led_cmd(const CAN_LedCmd_t *rx)
{
    switch (rx->cmd) {
    case CAN_LED_TEAM_RED:
        /* Team selection configures automatic effects; only test colours
           take manual ownership of the strip. Rendering stays in main. */
        led_cmd.source = LED_SRC_NORMAL;
        led_cmd.cmd    = CAN_LED_NORMAL;
        led_cmd.team   = 0;
        break;
    case CAN_LED_TEAM_BLUE:
        led_cmd.source = LED_SRC_NORMAL;
        led_cmd.cmd    = CAN_LED_NORMAL;
        led_cmd.team   = 1;
        break;
    case CAN_LED_HEAT_DATA:
        if (!Thermal_SetHeat(rx->heat_data, HAL_GetTick())) {
            return false;
        }
        /* The heat-bar display remains in automatic mode and follows the
           locally maintained thermal value as it subsequently cools. */
        led_cmd.source = LED_SRC_NORMAL;
        led_cmd.cmd = CAN_LED_NORMAL;
        led_cmd.heat_data = rx->heat_data;
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

/* ---- Rx callback -------------------------------------------------------- */
void CANProtocol_RxCallback(CAN_HandleTypeDef *hcan, uint32_t RxFifo)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t             rx_data[8];
    uint32_t            primask;

    if (HAL_CAN_GetRxMessage(hcan, RxFifo, &rx_header, rx_data) != HAL_OK)
        return;

    g_can_stats.rx_frames++;

    /* Calibration takes roughly 200 ms of I2C reads and delays. This ISR only
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
        if (shoot_report_request_count != 0xFFU) {
            shoot_report_request_count++;
        }
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
        primask = __get_PRIMASK();
        __disable_irq();
        {
            uint8_t next = (uint8_t)((led_ack_head + 1U) % LED_ACK_QUEUE_DEPTH);
            if (next != led_ack_tail) {
                memcpy(led_ack_queue[led_ack_head], rx_data, 8U);
                led_ack_head = next;
            } else if (led_ack_drop_count != UINT32_MAX) {
                /* ACKs are diagnostic responses; never overwrite an older
                 * response when CAN commands arrive faster than the main loop. */
                led_ack_drop_count++;
            }
        }
        __set_PRIMASK(primask);
        return;
    }

    g_can_stats.rx_unknown++;
}
