/**
 * @file cell_balancing_mgr.c
 * @brief Cell Balancing Manager Implementation
 * @details Passive cell balancing for lithium-ion battery pack equalization
 *
 * Safety Considerations:
 * - Temperature monitoring before and during balancing
 * - Voltage range checks to prevent over-discharge
 * - Maximum balancing time limits
 * - Emergency stop capability
 * - Cooldown periods between cycles
 */

#include "cell_balancing_mgr.h"
#include "bms_database.h"
#include "bms_config.h"
#include "mc33771_driver.h"
#include "timer.h"
#include "debug_info_mgr.h"
#include <string.h>

/*===========================================================================*/
/* Private Variables                                                         */
/*===========================================================================*/

static BalanceStatus_t balance_status;
static bool balancing_enabled = false;
static bool external_inhibit = false;
static uint32_t cooldown_start_ms = 0u;
static uint32_t last_evaluation_ms = 0u;

/* Cell voltage cache for balancing decisions */
static uint16_t cell_voltages[BMS_TOTAL_CELLS];

/*===========================================================================*/
/* Private Function Prototypes                                               */
/*===========================================================================*/

static void evaluate_balancing_need(void);
static void start_balancing(void);
static void stop_balancing(void);
static void update_active_balancing(void);
static uint8_t check_inhibit_conditions(void);
static void calculate_balance_targets(uint16_t *min_voltage, uint16_t *max_delta);
static uint8_t count_bits_set(uint16_t value);

/*===========================================================================*/
/* Public Function Implementations                                           */
/*===========================================================================*/

void CellBalance_Init(void) {
    /* Initialize status structure */
    memset(&balance_status, 0, sizeof(balance_status));
    balance_status.state = BALANCE_STATE_IDLE;

    balancing_enabled = false;
    external_inhibit = false;
    cooldown_start_ms = 0u;
    last_evaluation_ms = 0u;

    /* Ensure all balancing is disabled on startup */
    for (uint8_t slave = 0u; slave < BMS_NUM_SLAVES; slave++) {
        SlaveIF_DisableBalancing(slave);
        balance_status.cells_balancing[slave] = 0u;
    }

    DebugMgr_LogInfo("Cell Balancing Manager initialized");
}

void CellBalance_Update(void) {
    uint32_t current_time = TIMER_GetTick();

    /* Rate limit evaluation to every 100ms */
    if ((current_time - last_evaluation_ms) < 100u) {
        return;
    }
    last_evaluation_ms = current_time;

    /* Check for inhibit conditions */
    balance_status.inhibit_flags = check_inhibit_conditions();

    /* State machine */
    switch (balance_status.state) {
        case BALANCE_STATE_IDLE:
            if (balancing_enabled && (balance_status.inhibit_flags == BALANCE_INHIBIT_NONE)) {
                balance_status.state = BALANCE_STATE_EVALUATING;
            } else if (balance_status.inhibit_flags != BALANCE_INHIBIT_NONE) {
                balance_status.state = BALANCE_STATE_INHIBITED;
            }
            break;

        case BALANCE_STATE_EVALUATING:
            if (balance_status.inhibit_flags != BALANCE_INHIBIT_NONE) {
                balance_status.state = BALANCE_STATE_INHIBITED;
            } else {
                evaluate_balancing_need();
            }
            break;

        case BALANCE_STATE_ACTIVE:
            if (balance_status.inhibit_flags != BALANCE_INHIBIT_NONE) {
                stop_balancing();
                balance_status.state = BALANCE_STATE_INHIBITED;
            } else {
                update_active_balancing();
            }
            break;

        case BALANCE_STATE_COOLDOWN:
            if ((current_time - cooldown_start_ms) >= BALANCE_COOLDOWN_MS) {
                balance_status.state = BALANCE_STATE_IDLE;
            }
            break;

        case BALANCE_STATE_INHIBITED:
            if (balance_status.inhibit_flags == BALANCE_INHIBIT_NONE) {
                balance_status.state = BALANCE_STATE_IDLE;
            }
            break;

        default:
            balance_status.state = BALANCE_STATE_IDLE;
            break;
    }

    /* Update delta calculation for status reporting */
    uint16_t min_v, max_delta;
    calculate_balance_targets(&min_v, &max_delta);
    balance_status.max_delta_mV = max_delta;
}

bool CellBalance_IsBalancing(void) {
    return (balance_status.state == BALANCE_STATE_ACTIVE) &&
           (balance_status.total_cells_balancing > 0u);
}

BalanceState_t CellBalance_GetState(void) {
    return balance_status.state;
}

void CellBalance_GetStatus(BalanceStatus_t *status) {
    if (status != NULL) {
        memcpy(status, &balance_status, sizeof(BalanceStatus_t));
    }
}

void CellBalance_SetEnabled(bool enable) {
    if (enable && !balancing_enabled) {
        DebugMgr_LogInfo("Cell balancing enabled");
    } else if (!enable && balancing_enabled) {
        /* Stop any active balancing when disabled */
        if (balance_status.state == BALANCE_STATE_ACTIVE) {
            stop_balancing();
        }
        balance_status.state = BALANCE_STATE_IDLE;
        DebugMgr_LogInfo("Cell balancing disabled");
    }
    balancing_enabled = enable;
}

bool CellBalance_IsEnabled(void) {
    return balancing_enabled;
}

void CellBalance_EmergencyStop(void) {
    /* Immediately stop all balancing */
    for (uint8_t slave = 0u; slave < BMS_NUM_SLAVES; slave++) {
        SlaveIF_DisableBalancing(slave);
        balance_status.cells_balancing[slave] = 0u;
    }

    balance_status.total_cells_balancing = 0u;
    balance_status.state = BALANCE_STATE_INHIBITED;
    balance_status.inhibit_flags |= BALANCE_INHIBIT_SAFETY;

    DebugMgr_LogWarning("Cell balancing EMERGENCY STOP");
}

void CellBalance_SetExternalInhibit(bool inhibit) {
    external_inhibit = inhibit;

    if (inhibit && (balance_status.state == BALANCE_STATE_ACTIVE)) {
        stop_balancing();
        balance_status.state = BALANCE_STATE_INHIBITED;
    }
}

uint16_t CellBalance_GetMaxDelta(void) {
    return balance_status.max_delta_mV;
}

/*===========================================================================*/
/* Private Function Implementations                                          */
/*===========================================================================*/

/**
 * @brief Check all inhibit conditions
 * @return Bitmask of active inhibit conditions
 */
static uint8_t check_inhibit_conditions(void) {
    uint8_t inhibit = BALANCE_INHIBIT_NONE;

    /* Check external inhibit */
    if (external_inhibit) {
        inhibit |= BALANCE_INHIBIT_SAFETY;
    }

    /* Check if balancing is globally disabled */
    if (!balancing_enabled) {
        return inhibit;  /* Return early, no need to check further */
    }

    /* Get current cell data */
    const BMS_CellData_t *data = BMS_DB_GetAllData();

    if (data == NULL || !data->data_valid) {
        inhibit |= BALANCE_INHIBIT_COMM_FAULT;
        return inhibit;
    }

    /* Check voltage limits */
    uint16_t min_voltage = BMS_DB_GetMinCellVoltage();
    uint16_t max_voltage = BMS_DB_GetMaxCellVoltage();

    if (min_voltage < BALANCE_MIN_VOLTAGE_MV) {
        inhibit |= BALANCE_INHIBIT_VOLTAGE_LOW;
    }

    if (max_voltage > BALANCE_MAX_VOLTAGE_MV) {
        inhibit |= BALANCE_INHIBIT_VOLTAGE_HIGH;
    }

    /* Check temperature limits */
    int8_t max_temp = BMS_DB_GetMaxTemperature();

    /* Check all temperatures for min limit */
    for (uint8_t i = 0u; i < BMS_TOTAL_CELLS; i++) {
        int8_t temp = BMS_DB_GetCellTemperature(i);
        if (temp < BALANCE_MIN_TEMP_C) {
            inhibit |= BALANCE_INHIBIT_TEMP_LOW;
            break;
        }
    }

    if (max_temp > BALANCE_MAX_TEMP_C) {
        inhibit |= BALANCE_INHIBIT_TEMP_HIGH;
    }

    /* Check system status for faults */
    BMS_SystemStatus_t sys_status;
    BMS_DB_GetSystemStatus(&sys_status);

    if (sys_status.comm_fault) {
        inhibit |= BALANCE_INHIBIT_COMM_FAULT;
    }

    if (!sys_status.system_ok) {
        inhibit |= BALANCE_INHIBIT_SAFETY;
    }

    return inhibit;
}

/**
 * @brief Calculate balancing targets
 * @param min_voltage Output: minimum cell voltage
 * @param max_delta Output: maximum voltage delta
 */
static void calculate_balance_targets(uint16_t *min_voltage, uint16_t *max_delta) {
    uint16_t min_v = 0xFFFFu;
    uint16_t max_v = 0u;

    /* Read all cell voltages */
    for (uint8_t i = 0u; i < BMS_TOTAL_CELLS; i++) {
        cell_voltages[i] = BMS_DB_GetCellVoltage(i);

        if (cell_voltages[i] < min_v) {
            min_v = cell_voltages[i];
        }
        if (cell_voltages[i] > max_v) {
            max_v = cell_voltages[i];
        }
    }

    *min_voltage = min_v;
    *max_delta = (max_v > min_v) ? (max_v - min_v) : 0u;
}

/**
 * @brief Evaluate if balancing is needed and determine cells to balance
 */
static void evaluate_balancing_need(void) {
    uint16_t min_voltage;
    uint16_t max_delta;

    calculate_balance_targets(&min_voltage, &max_delta);

    /* Check if imbalance exceeds threshold */
    if (max_delta < BALANCE_START_THRESHOLD_MV) {
        /* No balancing needed */
        balance_status.state = BALANCE_STATE_IDLE;
        return;
    }

    /* Determine which cells need balancing */
    uint16_t balance_threshold = min_voltage + BALANCE_START_THRESHOLD_MV;
    uint8_t cells_to_balance = 0u;

    /* Clear previous balancing masks */
    for (uint8_t slave = 0u; slave < BMS_NUM_SLAVES; slave++) {
        balance_status.cells_balancing[slave] = 0u;
    }

    /* Identify cells above threshold */
    for (uint8_t i = 0u; i < BMS_TOTAL_CELLS; i++) {
        if (cell_voltages[i] > balance_threshold) {
            uint8_t slave_id = i / BMS_CELLS_PER_SLAVE;
            uint8_t cell_in_slave = i % BMS_CELLS_PER_SLAVE;

            balance_status.cells_balancing[slave_id] |= (1u << cell_in_slave);
            cells_to_balance++;
        }
    }

    if (cells_to_balance > 0u) {
        balance_status.target_voltage_mV = min_voltage;
        balance_status.total_cells_balancing = cells_to_balance;

        DebugMgr_LogInfo("Starting balancing: %d cells, target=%dmV, delta=%dmV",
                         cells_to_balance, min_voltage, max_delta);

        start_balancing();
    } else {
        balance_status.state = BALANCE_STATE_IDLE;
    }
}

/**
 * @brief Start balancing on identified cells
 */
static void start_balancing(void) {
    BalanceConfig_t config;
    config.balance_time_s = BALANCE_MAX_TIME_S;

    /* Send balancing commands to each slave */
    for (uint8_t slave = 0u; slave < BMS_NUM_SLAVES; slave++) {
        if (balance_status.cells_balancing[slave] != 0u) {
            config.balance_mask = balance_status.cells_balancing[slave];

            SlaveIF_Status_t status = SlaveIF_EnableBalancing(slave, &config);
            if (status != SLAVE_OK) {
                DebugMgr_LogError("Failed to enable balancing on slave %d", slave);
                /* Continue with other slaves */
            }
        }
    }

    balance_status.balance_start_ms = TIMER_GetTick();
    balance_status.state = BALANCE_STATE_ACTIVE;
}

/**
 * @brief Stop all active balancing
 */
static void stop_balancing(void) {
    /* Disable balancing on all slaves */
    for (uint8_t slave = 0u; slave < BMS_NUM_SLAVES; slave++) {
        SlaveIF_DisableBalancing(slave);
        balance_status.cells_balancing[slave] = 0u;
    }

    /* Update statistics */
    uint32_t balance_duration = TIMER_GetTick() - balance_status.balance_start_ms;
    balance_status.total_balance_time_s += (balance_duration / 1000u);

    balance_status.total_cells_balancing = 0u;

    /* Enter cooldown */
    cooldown_start_ms = TIMER_GetTick();
    balance_status.state = BALANCE_STATE_COOLDOWN;

    DebugMgr_LogInfo("Balancing stopped, entering cooldown");
}

/**
 * @brief Update during active balancing - monitor and adjust
 */
static void update_active_balancing(void) {
    uint32_t current_time = TIMER_GetTick();
    uint32_t elapsed_ms = current_time - balance_status.balance_start_ms;

    /* Check for maximum balancing time */
    if (elapsed_ms >= (BALANCE_MAX_TIME_S * 1000u)) {
        DebugMgr_LogInfo("Balancing max time reached");
        stop_balancing();
        return;
    }

    /* Re-evaluate cell voltages */
    uint16_t min_voltage;
    uint16_t max_delta;
    calculate_balance_targets(&min_voltage, &max_delta);

    /* Check if target reached (with hysteresis) */
    if (max_delta < BALANCE_STOP_HYSTERESIS_MV) {
        DebugMgr_LogInfo("Balancing complete: delta=%dmV", max_delta);
        stop_balancing();
        return;
    }

    /* Update target voltage (min may have changed) */
    balance_status.target_voltage_mV = min_voltage;
    balance_status.max_delta_mV = max_delta;

    /* Re-evaluate which cells need balancing */
    uint16_t balance_threshold = min_voltage + BALANCE_STOP_HYSTERESIS_MV;
    bool config_changed = false;

    for (uint8_t slave = 0u; slave < BMS_NUM_SLAVES; slave++) {
        uint16_t new_mask = 0u;

        for (uint8_t cell = 0u; cell < BMS_CELLS_PER_SLAVE; cell++) {
            uint8_t global_cell = (slave * BMS_CELLS_PER_SLAVE) + cell;

            if (cell_voltages[global_cell] > balance_threshold) {
                new_mask |= (1u << cell);
            }
        }

        /* Check if balancing configuration changed */
        if (new_mask != balance_status.cells_balancing[slave]) {
            balance_status.cells_balancing[slave] = new_mask;
            config_changed = true;

            /* Update slave balancing configuration */
            if (new_mask != 0u) {
                BalanceConfig_t config;
                config.balance_mask = new_mask;
                config.balance_time_s = BALANCE_MAX_TIME_S;
                SlaveIF_EnableBalancing(slave, &config);
            } else {
                SlaveIF_DisableBalancing(slave);
            }
        }
    }

    /* Recount total cells balancing */
    if (config_changed) {
        balance_status.total_cells_balancing = 0u;
        for (uint8_t slave = 0u; slave < BMS_NUM_SLAVES; slave++) {
            balance_status.total_cells_balancing +=
                count_bits_set(balance_status.cells_balancing[slave]);
        }

        /* Check if all cells are now balanced */
        if (balance_status.total_cells_balancing == 0u) {
            DebugMgr_LogInfo("All cells balanced");
            stop_balancing();
        }
    }
}

/**
 * @brief Count number of bits set in a 16-bit value
 */
static uint8_t count_bits_set(uint16_t value) {
    uint8_t count = 0u;
    while (value != 0u) {
        count += (value & 1u);
        value >>= 1;
    }
    return count;
}
