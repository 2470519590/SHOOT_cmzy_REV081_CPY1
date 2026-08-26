/**
 ******************************************************************************
 * @file           : reliability.c
 * @brief          : Weak/strong fault tracking and reset-surviving fault note.
 ******************************************************************************
 */

#include "reliability.h"

#define FAULT_RECORD_MAGIC          0x52464C54UL /* "RFLT" */
#define SENSOR_FAIL_LIMIT           3U
#define BUSOFF_ALERT_MS             2000U

typedef struct {
    uint32_t magic;
    uint32_t strong_mask;
    uint32_t pc;
    uint32_t lr;
    uint32_t xpsr;
    uint32_t checksum;
} FaultRecord_t;

/* This section is deliberately outside .bss and survives software/IWDG reset. */
static FaultRecord_t g_fault_record __attribute__((section(".noinit")));

static uint8_t pending_strong_mask;
static uint8_t weak_mask;
static uint8_t front_fail_count;
static uint8_t rear_fail_count;
static uint32_t last_queue_dropped;
static uint32_t queue_fault_last_tick;
static bool strong_reset_pending;
static uint32_t strong_reset_tick;

static uint32_t fault_checksum(const FaultRecord_t *record)
{
    return record->magic ^ record->strong_mask ^ record->pc ^
           record->lr ^ record->xpsr ^ 0xA5C37E19UL;
}

static void save_strong_fault(uint8_t mask, uint32_t pc, uint32_t lr, uint32_t xpsr)
{
    g_fault_record.magic = FAULT_RECORD_MAGIC;
    g_fault_record.strong_mask = mask;
    g_fault_record.pc = pc;
    g_fault_record.lr = lr;
    g_fault_record.xpsr = xpsr;
    g_fault_record.checksum = fault_checksum(&g_fault_record);
}

void Reliability_EarlyInit(void)
{
    uint32_t reset_flags = RCC->CSR;

    pending_strong_mask = 0;
    if (g_fault_record.magic == FAULT_RECORD_MAGIC &&
        g_fault_record.checksum == fault_checksum(&g_fault_record)) {
        pending_strong_mask |= (uint8_t)g_fault_record.strong_mask;
    }
    if ((reset_flags & RCC_CSR_IWDGRSTF) != 0U) {
        pending_strong_mask |= REL_STRONG_WATCHDOG;
    }

    /* Consume the retained record. A new fault overwrites it before reset. */
    g_fault_record.magic = 0U;
    g_fault_record.checksum = 0U;
    RCC->CSR |= RCC_CSR_RMVF;
}

uint8_t Reliability_GetPendingStrongMask(void)
{
    return pending_strong_mask;
}

void Reliability_SetWeakFault(uint8_t mask, bool active)
{
    if (active) weak_mask |= mask;
    else        weak_mask &= (uint8_t)~mask;
}

void Reliability_ObserveSensors(bool front_ok, bool rear_ok)
{
    if (front_ok) {
        front_fail_count = 0U;
        Reliability_SetWeakFault(REL_WEAK_FRONT_SENSOR, false);
    } else if (front_fail_count < SENSOR_FAIL_LIMIT) {
        front_fail_count++;
        if (front_fail_count >= SENSOR_FAIL_LIMIT) {
            Reliability_SetWeakFault(REL_WEAK_FRONT_SENSOR, true);
        }
    }

    if (rear_ok) {
        rear_fail_count = 0U;
        Reliability_SetWeakFault(REL_WEAK_REAR_SENSOR, false);
    } else if (rear_fail_count < SENSOR_FAIL_LIMIT) {
        rear_fail_count++;
        if (rear_fail_count >= SENSOR_FAIL_LIMIT) {
            Reliability_SetWeakFault(REL_WEAK_REAR_SENSOR, true);
        }
    }
}

void Reliability_ObserveEventQueueDropped(uint32_t dropped_count)
{
    uint32_t now = HAL_GetTick();

    if (dropped_count != last_queue_dropped) {
        last_queue_dropped = dropped_count;
        queue_fault_last_tick = now;
        Reliability_SetWeakFault(REL_WEAK_EVENT_QUEUE, true);
    } else if ((weak_mask & REL_WEAK_EVENT_QUEUE) != 0U &&
               (now - queue_fault_last_tick) >= 1000U) {
        Reliability_SetWeakFault(REL_WEAK_EVENT_QUEUE, false);
    }
}

uint8_t Reliability_GetWeakMask(void)
{
    return weak_mask;
}

void Reliability_RequestStrongFault(uint8_t mask)
{
    if (!strong_reset_pending) {
        save_strong_fault(mask, 0U, 0U, 0U);
        strong_reset_pending = true;
        strong_reset_tick = HAL_GetTick();
    }
}

bool Reliability_IsFaultAlertActive(void)
{
    return strong_reset_pending || (weak_mask != 0U);
}

bool Reliability_ShouldResetNow(void)
{
    return strong_reset_pending &&
           ((HAL_GetTick() - strong_reset_tick) >= BUSOFF_ALERT_MS);
}

void Reliability_HandleHardFault(const uint32_t *stack_frame)
{
    uint32_t pc = 0U;
    uint32_t lr = 0U;
    uint32_t xpsr = 0U;

    if (stack_frame != NULL) {
        lr = stack_frame[5];
        pc = stack_frame[6];
        xpsr = stack_frame[7];
    }
    save_strong_fault(REL_STRONG_HARDFAULT, pc, lr, xpsr);
    __DSB();
    NVIC_SystemReset();
    while (1) { }
}
