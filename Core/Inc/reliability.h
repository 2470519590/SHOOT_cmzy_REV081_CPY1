/**
 ******************************************************************************
 * @file           : reliability.h
 * @brief          : Minimal competition reliability state and reset handling.
 ******************************************************************************
 */
#ifndef __RELIABILITY_H
#define __RELIABILITY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* CAN 0x211: weak-fault mask. */
#define REL_WEAK_FRONT_SENSOR       (1U << 0)
#define REL_WEAK_REAR_SENSOR        (1U << 1)
#define REL_WEAK_SHOOT_DETECT       (1U << 2)
#define REL_WEAK_TASK_TIMEOUT       (1U << 3)
#define REL_WEAK_EVENT_QUEUE        (1U << 4)
#define REL_WEAK_OTHER              (1U << 7)

/* CAN 0x210: strong-fault mask. */
#define REL_STRONG_HARDFAULT        (1U << 0)
#define REL_STRONG_WATCHDOG         (1U << 1)
#define REL_STRONG_CAN_BUSOFF       (1U << 2)
#define REL_STRONG_OTHER_RESET      (1U << 3)
#define REL_STRONG_OTHER            (1U << 7)

void Reliability_EarlyInit(void);
uint8_t Reliability_GetPendingStrongMask(void);

void Reliability_ObserveSensors(bool front_ok, bool rear_ok);
void Reliability_ObserveEventQueueDropped(uint32_t dropped_count);
void Reliability_SetWeakFault(uint8_t mask, bool active);
uint8_t Reliability_GetWeakMask(void);

void Reliability_RequestStrongFault(uint8_t mask);
bool Reliability_IsFaultAlertActive(void);
bool Reliability_ShouldResetNow(void);

/* Called from the naked Cortex-M0 HardFault handler with its exception stack. */
void Reliability_HandleHardFault(const uint32_t *stack_frame);

#ifdef __cplusplus
}
#endif

#endif /* __RELIABILITY_H */
