/**
 * @file cell_balancing_mgr.h
 * @brief Cell Balancing Manager - Application Layer
 * @details Passive cell balancing algorithm for lithium-ion battery pack
 *
 * Balancing Strategy:
 * - Passive balancing: discharge high cells via bleed resistors
 * - Target: equalize all cells to lowest cell voltage
 * - Safety: temperature and voltage limits before balancing
 *
 * Algorithm:
 * 1. Find minimum cell voltage in pack
 * 2. Identify cells exceeding min + threshold
 * 3. Enable balancing on those cells
 * 4. Monitor until delta < hysteresis
 */

#ifndef APP_CELL_BALANCING_MGR_H
#define APP_CELL_BALANCING_MGR_H

#include <stdint.h>
#include <stdbool.h>
#include "bms_config.h"

/*===========================================================================*/
/* Balancing Configuration                                                   */
/*===========================================================================*/

/* Balancing activation threshold (mV) - start balancing if cell > min + threshold */
#define BALANCE_START_THRESHOLD_MV      20u

/* Balancing deactivation hysteresis (mV) - stop when cell < min + hysteresis */
#define BALANCE_STOP_HYSTERESIS_MV      10u

/* Maximum balancing time per cycle (seconds) */
#define BALANCE_MAX_TIME_S              60u

/* Minimum cell voltage to allow balancing (mV) */
#define BALANCE_MIN_VOLTAGE_MV          3200u

/* Maximum cell voltage for balancing (mV) - don't balance during CV charging */
#define BALANCE_MAX_VOLTAGE_MV          4150u

/* Temperature limits for balancing */
#define BALANCE_MIN_TEMP_C              0
#define BALANCE_MAX_TEMP_C              45

/* Minimum time between balance cycles (ms) */
#define BALANCE_COOLDOWN_MS             5000u

/*===========================================================================*/
/* Data Structures                                                           */
/*===========================================================================*/

/**
 * @brief Balancing state enumeration
 */
typedef enum {
    BALANCE_STATE_IDLE = 0,         /**< No balancing active */
    BALANCE_STATE_EVALUATING,       /**< Checking if balancing needed */
    BALANCE_STATE_ACTIVE,           /**< Balancing in progress */
    BALANCE_STATE_COOLDOWN,         /**< Cooldown period between cycles */
    BALANCE_STATE_INHIBITED         /**< Balancing disabled (safety condition) */
} BalanceState_t;

/**
 * @brief Balancing inhibit reasons
 */
typedef enum {
    BALANCE_INHIBIT_NONE = 0x00,
    BALANCE_INHIBIT_VOLTAGE_LOW = 0x01,     /**< Cell voltage too low */
    BALANCE_INHIBIT_VOLTAGE_HIGH = 0x02,    /**< Cell voltage too high */
    BALANCE_INHIBIT_TEMP_LOW = 0x04,        /**< Temperature too low */
    BALANCE_INHIBIT_TEMP_HIGH = 0x08,       /**< Temperature too high */
    BALANCE_INHIBIT_CHARGING = 0x10,        /**< Active charging detected */
    BALANCE_INHIBIT_COMM_FAULT = 0x20,      /**< Communication fault */
    BALANCE_INHIBIT_SAFETY = 0x40           /**< FuSa supervisor inhibit */
} BalanceInhibit_t;

/**
 * @brief Balancing status structure
 */
typedef struct {
    BalanceState_t state;               /**< Current balancing state */
    uint8_t inhibit_flags;              /**< Active inhibit reasons */
    uint16_t cells_balancing[BMS_NUM_SLAVES];  /**< Bitmask per slave */
    uint8_t total_cells_balancing;      /**< Count of cells being balanced */
    uint16_t target_voltage_mV;         /**< Current target voltage */
    uint16_t max_delta_mV;              /**< Maximum cell-to-cell delta */
    uint32_t balance_start_ms;          /**< Timestamp of balance start */
    uint32_t total_balance_time_s;      /**< Cumulative balancing time */
} BalanceStatus_t;

/*===========================================================================*/
/* Function Prototypes                                                       */
/*===========================================================================*/

/**
 * @brief Initialize the cell balancing manager
 */
void CellBalance_Init(void);

/**
 * @brief Periodic update function - call from main loop
 * @details Evaluates cell voltages and controls balancing
 */
void CellBalance_Update(void);

/**
 * @brief Check if any cell is currently balancing
 * @return true if balancing is active
 */
bool CellBalance_IsBalancing(void);

/**
 * @brief Get the current balancing state
 * @return Current balance state
 */
BalanceState_t CellBalance_GetState(void);

/**
 * @brief Get detailed balancing status
 * @param status Pointer to status structure to fill
 */
void CellBalance_GetStatus(BalanceStatus_t *status);

/**
 * @brief Enable/disable balancing globally
 * @param enable true to enable, false to disable
 */
void CellBalance_SetEnabled(bool enable);

/**
 * @brief Check if balancing is enabled
 * @return true if balancing is enabled
 */
bool CellBalance_IsEnabled(void);

/**
 * @brief Force stop all balancing immediately
 * @details Called by FuSa supervisor on fault detection
 */
void CellBalance_EmergencyStop(void);

/**
 * @brief Set external inhibit condition
 * @param inhibit true to inhibit balancing
 * @details Used by charging manager or FuSa supervisor
 */
void CellBalance_SetExternalInhibit(bool inhibit);

/**
 * @brief Get maximum cell voltage delta in pack
 * @return Delta between max and min cell voltage (mV)
 */
uint16_t CellBalance_GetMaxDelta(void);

#endif /* APP_CELL_BALANCING_MGR_H */
