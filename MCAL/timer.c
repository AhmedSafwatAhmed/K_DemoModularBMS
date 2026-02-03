/**
 * @file timer.c
 * @brief Timer/SysTick Driver Implementation for KL25Z
 * @details SysTick timer configured for 1ms system tick
 */

#include "timer.h"

/* System clock frequency (48MHz) */
#define SYSTEM_CLOCK_HZ         48000000u

/* SysTick reload value for 1ms tick */
#define SYSTICK_RELOAD_1MS      (SYSTEM_CLOCK_HZ / 1000u - 1u)

/*===========================================================================*/
/* Private Variables                                                         */
/*===========================================================================*/

/** System tick counter - incremented every 1ms */
static volatile uint32_t system_tick_count = 0u;

/*===========================================================================*/
/* Interrupt Handler                                                         */
/*===========================================================================*/

/**
 * @brief SysTick Interrupt Handler
 * Called every 1ms when SysTick counter reaches zero
 */
void SysTick_Handler(void) {
    system_tick_count++;
}

/*===========================================================================*/
/* Public Function Implementations                                           */
/*===========================================================================*/

TIMER_Status_t TIMER_Init(void) {
    /* Disable SysTick during configuration */
    SYST_CSR = 0u;

    /* Set reload value for 1ms tick
     * Reload = (SystemClock / 1000) - 1
     * For 48MHz: Reload = 48000 - 1 = 47999
     */
    SYST_RVR = SYSTICK_RELOAD_1MS;

    /* Clear current value */
    SYST_CVR = 0u;

    /* Enable SysTick with:
     * - Core clock source
     * - Interrupt enabled
     * - Counter enabled
     */
    SYST_CSR = SYST_CSR_CLKSOURCE_MASK |
               SYST_CSR_TICKINT_MASK |
               SYST_CSR_ENABLE_MASK;

    return TIMER_OK;
}

uint32_t TIMER_GetTick(void) {
    return system_tick_count;
}

void TIMER_DelayMs(uint32_t delay_ms) {
    uint32_t start_tick = system_tick_count;

    /* Wait until required time has elapsed */
    while ((system_tick_count - start_tick) < delay_ms) {
        /* Wait - SysTick handler increments counter */
        __asm volatile ("nop");
    }
}

void TIMER_DelayUs(uint32_t delay_us) {
    /*
     * Approximate microsecond delay using CPU cycles
     * At 48MHz, 1us = 48 cycles
     * Loop overhead is roughly 4 cycles per iteration
     * So we need about 12 iterations per microsecond
     */
    volatile uint32_t count = (delay_us * 12u);

    while (count > 0u) {
        count--;
    }
}

bool TIMER_IsTimeoutElapsed(uint32_t start_tick, uint32_t timeout_ms) {
    uint32_t elapsed = system_tick_count - start_tick;
    return (elapsed >= timeout_ms);
}

uint32_t TIMER_GetElapsed(uint32_t start_tick) {
    return (system_tick_count - start_tick);
}
