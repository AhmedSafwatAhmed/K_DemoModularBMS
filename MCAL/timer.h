/**
 * @file timer.h
 * @brief Timer/SysTick Driver for KL25Z
 * @details SysTick-based timing services for the BMS application
 *
 * Uses ARM Cortex-M0+ SysTick timer for:
 * - System tick counter (1ms resolution)
 * - Blocking delays
 * - Timeout management
 *
 * SysTick Configuration:
 * - Clock source: Core clock (48MHz)
 * - Tick period: 1ms
 * - 24-bit down counter
 */

#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>
#include <stdbool.h>

/*===========================================================================*/
/* SysTick Register Definitions                                              */
/*===========================================================================*/

#define SYST_CSR                (*(volatile uint32_t *)0xE000E010u)  /* Control/Status */
#define SYST_RVR                (*(volatile uint32_t *)0xE000E014u)  /* Reload Value */
#define SYST_CVR                (*(volatile uint32_t *)0xE000E018u)  /* Current Value */

/* SysTick Control Register Bits */
#define SYST_CSR_ENABLE_MASK    (1u << 0)   /* Counter Enable */
#define SYST_CSR_TICKINT_MASK   (1u << 1)   /* Exception Enable */
#define SYST_CSR_CLKSOURCE_MASK (1u << 2)   /* Clock Source (1=Core) */
#define SYST_CSR_COUNTFLAG_MASK (1u << 16)  /* Count Flag */

/*===========================================================================*/
/* Timer Status Codes                                                        */
/*===========================================================================*/

typedef enum {
    TIMER_OK = 0,
    TIMER_ERROR = 1,
    TIMER_TIMEOUT = 2
} TIMER_Status_t;

/*===========================================================================*/
/* Function Prototypes                                                       */
/*===========================================================================*/

/**
 * @brief Initialize SysTick timer for 1ms tick
 * @return TIMER_OK on success
 *
 * Configures SysTick to generate 1ms interrupts.
 * Must be called before using any timing functions.
 */
TIMER_Status_t TIMER_Init(void);

/**
 * @brief Get current system tick count
 * @return Tick count in milliseconds since init
 *
 * Wraps around at UINT32_MAX (~49.7 days)
 */
uint32_t TIMER_GetTick(void);

/**
 * @brief Blocking delay in milliseconds
 * @param delay_ms Delay duration in milliseconds
 *
 * Note: Blocks CPU. Use for initialization only.
 * For application timing, use TIMER_GetTick() comparisons.
 */
void TIMER_DelayMs(uint32_t delay_ms);

/**
 * @brief Blocking delay in microseconds
 * @param delay_us Delay duration in microseconds
 *
 * Note: Approximate timing, uses CPU cycling.
 * Accuracy depends on optimization level.
 */
void TIMER_DelayUs(uint32_t delay_us);

/**
 * @brief Check if timeout has elapsed
 * @param start_tick Starting tick value
 * @param timeout_ms Timeout duration in milliseconds
 * @return true if timeout elapsed, false otherwise
 *
 * Handles tick counter wrap-around correctly.
 */
bool TIMER_IsTimeoutElapsed(uint32_t start_tick, uint32_t timeout_ms);

/**
 * @brief Get elapsed time since a start tick
 * @param start_tick Starting tick value
 * @return Elapsed time in milliseconds
 *
 * Handles tick counter wrap-around correctly.
 */
uint32_t TIMER_GetElapsed(uint32_t start_tick);

/**
 * @brief SysTick interrupt handler
 *
 * Called automatically by hardware on each SysTick interrupt.
 * Increments the internal tick counter.
 */
void SysTick_Handler(void);

#endif /* TIMER_H */
