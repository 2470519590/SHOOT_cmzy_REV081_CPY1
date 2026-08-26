/**
  ******************************************************************************
  * @file           : can_protocol.h
  * @brief          : CAN referee-system slave protocol
  * @description    : Rx: team color, heat data, test commands.
  *                   Tx: shot count, speed, barrel status, heat, sensor data.
  *                   Message IDs follow docs/CAN_PROTOCOL.md.
  ******************************************************************************
  */
#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "shoot_detect.h"
#include <stdbool.h>

/* ========================== CAN Message IDs ================================= */

#define CAN_HEARTBEAT_ID           0x200   /* Tx: debug heartbeat, once per sec   */
#define CAN_STRONG_FAULT_ID        0x210   /* Tx: post-reset strong-fault mask    */
#define CAN_WEAK_FAULT_ID          0x211   /* Tx: weak-fault mask                 */
#define CAN_BOOT_ID                0x212   /* Tx: initialization complete         */
#define CAN_CALIBRATE_REQUEST_ID   0x220   /* Rx: request sensor recalibration    */
#define CAN_CALIBRATE_ACK_ID       0x221   /* Tx: sensor recalibration result     */
#define CAN_SHOT_EVENT_ID          0x230   /* Tx: one frame for each valid shot   */
#define CAN_REFEREE_QUERY_ID       0x231   /* Rx: request shooting status         */
#define CAN_BOARD_RESPONSE_ID      0x232   /* Tx: shooting status response        */
#define CAN_LED_CMD_ID             0x233   /* Rx: LED/team/heat control           */
#define CAN_LED_ACK_ID             0x234   /* Tx: echoed LED command ACK          */

/* ---- CAN_LED_CMD_ID sub-commands (data[0]) --------------------------------- */
#define CAN_LED_TEAM_RED           0x01    /* set team = red                    */
#define CAN_LED_TEAM_BLUE          0x02    /* set team = blue                   */
#define CAN_LED_HEAT_DATA          0x03    /* data[1] = heat value 0-255        */
#define CAN_LED_TEST_GREEN         0x10    /* all LEDs green (factory test)     */
#define CAN_LED_TEST_RED           0x11    /* all LEDs red                      */
#define CAN_LED_TEST_BLUE          0x12    /* all LEDs blue                     */
#define CAN_LED_TEST_OFF           0x1F    /* all LEDs off                      */
#define CAN_LED_NORMAL             0x00    /* resume auto mode                  */

/* ========================== Tx Frame (8 bytes to 0x232) ===================== */

typedef struct __attribute__((packed)) {
    uint32_t shot_count;          /* [ 3: 0] cumulative valid shots             */
    uint16_t last_speed_cmps;     /* [ 5: 4] last speed × 100 (cm/s)           */
    uint8_t  barrel_mask;         /* [ 6   ] projectiles in barrel, bits[4:0]  */
    uint8_t  heat_level;          /* [ 7   ] current heat 0-255                */
} CAN_ShootReport_t;

/* ========================== Rx Frame (8 bytes from 0x233) =================== */

typedef struct __attribute__((packed)) {
    uint8_t  cmd;                 /* [0] sub-command (CAN_LED_xxx)             */
    uint8_t  heat_data;           /* [1] heat value (for CAN_LED_HEAT_DATA)    */
    uint8_t  reserved[6];         /* [7:2]                                     */
} CAN_LedCmd_t;

/* ========================== Internal State ================================== */

typedef struct {
    uint32_t shot_count;
    uint32_t front_int_count;
    uint32_t rear_int_count;
    float    last_speed_mps;
    uint8_t  barrel_mask;
    uint8_t  heat_level;
    uint16_t front_prox;
    uint16_t rear_prox;
} ShootData_Report_t;

/* Runtime CAN diagnostics; inspect g_can_stats in the debugger. */
typedef struct {
    volatile uint32_t rx_frames;
    volatile uint32_t rx_calibrate_request;
    volatile uint32_t rx_query_0x231;
    volatile uint32_t rx_led_cmd;
    volatile uint32_t tx_led_ack_ok;
    volatile uint32_t tx_led_ack_fail;
    volatile uint32_t rx_unknown;
    volatile uint32_t tx_status_ok;
    volatile uint32_t tx_status_fail;
    volatile uint32_t tx_heartbeat_ok;
    volatile uint32_t tx_heartbeat_fail;
    volatile uint32_t error_callbacks;
    volatile uint32_t error_code;
    volatile uint32_t error_events;
    volatile uint32_t last_error_tick;
    volatile uint8_t  last_error_lec;
    volatile uint8_t  tx_error_counter;
    volatile uint8_t  rx_error_counter;
    volatile uint8_t  bus_off;
    volatile uint8_t  state;
} CAN_ErrorStats_t;

extern CAN_ErrorStats_t g_can_stats;

typedef enum {
    LED_SRC_NORMAL = 0,          /* auto — follow shoot detection              */
    LED_SRC_DEBUG,                /* manual via CAN or debugger                 */
} LedCmdSource_t;

typedef struct {
    LedCmdSource_t source;
    uint8_t        cmd;
    uint8_t        heat_data;
    uint8_t        team;          /* 0=red, 1=blue                             */
} LedCommand_t;

/* ========================== Public API ====================================== */

void CANProtocol_Init(CAN_HandleTypeDef *hcan);
void CANProtocol_UpdateData(const ShootData_Report_t *data);
HAL_StatusTypeDef CANProtocol_SendHeartbeat(void);
HAL_StatusTypeDef CANProtocol_SendShotEvent(const ShootEvent_t *event);
HAL_StatusTypeDef CANProtocol_SendWeakFault(uint8_t weak_mask);
HAL_StatusTypeDef CANProtocol_SendStrongFault(uint8_t strong_mask);
HAL_StatusTypeDef CANProtocol_SendBoot(void);
HAL_StatusTypeDef CANProtocol_SendCalibrationAck(uint8_t status);
bool CANProtocol_TakeCalibrationRequest(void);
void CANProtocol_RxCallback(CAN_HandleTypeDef *hcan, uint32_t RxFifo);
const LedCommand_t *CANProtocol_GetLedCommand(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_PROTOCOL_H */
