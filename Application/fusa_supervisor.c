/**
 * @file fusa_supervisor.c
 * @brief Functional Safety Supervisor Implementation
 * @details ISO 26262 ASIL-C compliant safety monitoring for EV-BMS
 *
 * Safety Concept:
 * 1. Detect faults within FTTI (Fault Tolerant Time Interval)
 * 2. Transition to safe state when critical fault detected
 * 3. Log all safety-relevant events
 * 4. Prevent unsafe operations
 */

#include "fusa_supervisor.h"
#include "bms_database.h"
#include "bms_config.h"
#include "debug_info_mgr.h"
#include "gpio.h"
#include "timer.h"
#include <string.h>

/*===========================================================================*/
/* Configuration                                                             */
/*===========================================================================*/

#define FUSA_FAULT_LOG_SIZE         16u
#define FUSA_RAM_TEST_SIZE          32u
#define FUSA_RAM_PATTERN_A          0x55u
#define FUSA_RAM_PATTERN_B          0xAAu
#define FUSA_CRC_POLYNOMIAL         0x07u

/*===========================================================================*/
/* Private Types                                                             */
/*===========================================================================*/

/**
 * @brief Previous measurement storage for gradient calculation
 */
typedef struct {
    uint16_t cell_voltages[BMS_TOTAL_CELLS];
    int8_t cell_temps[BMS_TOTAL_CELLS];
    uint32_t timestamp_ms;
    bool valid;
} FuSa_PreviousMeasurement_t;

/*===========================================================================*/
/* Private Variables                                                         */
/*===========================================================================*/

static FuSa_State_t current_state = FUSA_STATE_INIT;
static FuSa_FaultEntry_t fault_log[FUSA_FAULT_LOG_SIZE];
static uint8_t fault_log_index = 0u;
static uint8_t active_fault_count = 0u;

static FuSa_PreviousMeasurement_t prev_measurement;
static uint32_t last_check_time_ms = 0u;
static uint32_t init_time_ms = 0u;
static uint32_t alive_counter = 0u;
static uint32_t last_alive_time_ms = 0u;

/* Debounce counters for fault confirmation */
static uint8_t fault_debounce[FUSA_FAULT_COUNT];

/* RAM test area (must be in RAM, not optimized away) */
static volatile uint8_t ram_test_area[FUSA_RAM_TEST_SIZE];

/* Safety flags */
static bool contactor_allowed = false;
static bool charging_allowed = false;
static bool discharging_allowed = false;

/*===========================================================================*/
/* Private Function Prototypes                                               */
/*===========================================================================*/

static void log_fault(FuSa_FaultType_t type, FuSa_Severity_t severity,
                      uint16_t data, uint8_t cell_idx);
static bool check_voltage_plausibility(void);
static bool check_temperature_plausibility(void);
static bool check_voltage_gradient(void);
static bool check_temperature_gradient(void);
static bool check_critical_limits(void);
static bool check_data_freshness(void);
static bool check_alive_supervision(void);
static void update_state_machine(void);
static void enter_safe_state(void);
static void open_contactors(void);
static void update_operation_permissions(void);
static FuSa_Severity_t get_fault_severity(FuSa_FaultType_t type);

/*===========================================================================*/
/* Public Function Implementations                                           */
/*===========================================================================*/

void FuSa_Init(void) {
    DebugMgr_LogInfo("FuSa: Initializing safety supervisor...");

    /* Clear fault log */
    memset(fault_log, 0, sizeof(fault_log));
    memset(fault_debounce, 0, sizeof(fault_debounce));
    fault_log_index = 0u;
    active_fault_count = 0u;

    /* Clear previous measurements */
    memset(&prev_measurement, 0, sizeof(prev_measurement));
    prev_measurement.valid = false;

    /* Record initialization time */
    init_time_ms = TIMER_GetTick();
    last_check_time_ms = init_time_ms;
    last_alive_time_ms = init_time_ms;
    alive_counter = 0u;

    /* Perform RAM integrity check */
    if (!FuSa_CheckRAMIntegrity()) {
        DebugMgr_LogError("FuSa: RAM integrity check FAILED!");
        log_fault(FUSA_FAULT_RAM_CORRUPTION, FUSA_SEVERITY_CRITICAL, 0u, 0u);
        current_state = FUSA_STATE_SAFE;
    } else {
        DebugMgr_LogInfo("FuSa: RAM integrity check passed");
        current_state = FUSA_STATE_NORMAL;
    }

    /* Initialize safety outputs */
    contactor_allowed = false;
    charging_allowed = false;
    discharging_allowed = false;

    /* Ensure contactors are open at startup */
    open_contactors();

    DebugMgr_LogInfo("FuSa: Supervisor initialized, state=NORMAL");
}

bool FuSa_PerformSafetyChecks(void) {
    bool all_checks_passed = true;
    uint32_t current_time = TIMER_GetTick();

    /* Verify we're being called within FTTI */
    uint32_t time_since_last = current_time - last_check_time_ms;
    if (time_since_last > BMS_FUSA_FTTI_MS && last_check_time_ms != 0u) {
        DebugMgr_LogWarning("FuSa: FTTI violation detected!");
        log_fault(FUSA_FAULT_ALIVE_TIMEOUT, FUSA_SEVERITY_DEGRADED,
                  (uint16_t)time_since_last, 0u);
        all_checks_passed = false;
    }
    last_check_time_ms = current_time;

    /* 1. Voltage Plausibility Check */
    if (!check_voltage_plausibility()) {
        all_checks_passed = false;
    }

    /* 2. Temperature Plausibility Check */
    if (!check_temperature_plausibility()) {
        all_checks_passed = false;
    }

    /* 3. Voltage Gradient Check (rate of change) */
    if (prev_measurement.valid) {
        if (!check_voltage_gradient()) {
            all_checks_passed = false;
        }

        /* 4. Temperature Gradient Check */
        if (!check_temperature_gradient()) {
            all_checks_passed = false;
        }
    }

    /* 5. Critical Limits Check (OV/UV/OT) */
    if (!check_critical_limits()) {
        all_checks_passed = false;
    }

    /* 6. Data Freshness Check */
    if (!check_data_freshness()) {
        all_checks_passed = false;
    }

    /* 7. Alive Supervision Check */
    if (!check_alive_supervision()) {
        all_checks_passed = false;
    }

    /* 8. Periodic RAM Check (every 10 seconds) */
    static uint32_t last_ram_check = 0u;
    if ((current_time - last_ram_check) > 10000u) {
        if (!FuSa_CheckRAMIntegrity()) {
            log_fault(FUSA_FAULT_RAM_CORRUPTION, FUSA_SEVERITY_CRITICAL, 0u, 0u);
            all_checks_passed = false;
        }
        last_ram_check = current_time;
    }

    /* Store current measurements for next gradient calculation */
    for (uint8_t i = 0u; i < BMS_TOTAL_CELLS; i++) {
        prev_measurement.cell_voltages[i] = BMS_DB_GetCellVoltage(i);
        prev_measurement.cell_temps[i] = BMS_DB_GetCellTemperature(i);
    }
    prev_measurement.timestamp_ms = current_time;
    prev_measurement.valid = true;

    /* Update state machine based on fault status */
    update_state_machine();

    /* Update operation permissions */
    update_operation_permissions();

    return all_checks_passed;
}

bool FuSa_GetSafetyStatus(void) {
    return (current_state == FUSA_STATE_NORMAL ||
            current_state == FUSA_STATE_DEGRADED);
}

void FuSa_GetDetailedStatus(FuSa_Status_t *status) {
    if (status == NULL) return;

    status->current_state = current_state;
    status->active_fault_count = active_fault_count;
    status->last_check_ms = last_check_time_ms;
    status->uptime_ms = TIMER_GetTick() - init_time_ms;
    status->contactor_allowed = contactor_allowed;
    status->charging_allowed = charging_allowed;
    status->discharging_allowed = discharging_allowed;
}

FuSa_State_t FuSa_GetState(void) {
    return current_state;
}

bool FuSa_IsOperationAllowed(uint8_t operation) {
    switch (operation) {
        case 0: return contactor_allowed;
        case 1: return charging_allowed;
        case 2: return discharging_allowed;
        default: return false;
    }
}

void FuSa_RequestSafeState(FuSa_FaultType_t fault_type) {
    DebugMgr_LogError("FuSa: Safe state requested!");
    log_fault(fault_type, FUSA_SEVERITY_CRITICAL, 0u, 0u);
    enter_safe_state();
}

bool FuSa_AttemptRecovery(void) {
    /* Only allow recovery if no critical faults active */
    if (active_fault_count > 0u) {
        DebugMgr_LogWarning("FuSa: Recovery denied - faults still active");
        return false;
    }

    if (current_state == FUSA_STATE_EMERGENCY) {
        DebugMgr_LogWarning("FuSa: Recovery denied - emergency state");
        return false;
    }

    /* Verify RAM integrity before recovery */
    if (!FuSa_CheckRAMIntegrity()) {
        DebugMgr_LogError("FuSa: Recovery denied - RAM check failed");
        return false;
    }

    DebugMgr_LogInfo("FuSa: Attempting recovery...");
    current_state = FUSA_STATE_NORMAL;
    update_operation_permissions();

    return true;
}

bool FuSa_ClearFault(FuSa_FaultType_t fault_type) {
    if (fault_type >= FUSA_FAULT_COUNT) {
        return false;
    }

    bool cleared = false;
    for (uint8_t i = 0u; i < FUSA_FAULT_LOG_SIZE; i++) {
        if (fault_log[i].fault_type == fault_type && fault_log[i].active) {
            fault_log[i].active = false;
            cleared = true;
        }
    }

    if (cleared) {
        active_fault_count--;
        fault_debounce[fault_type] = 0u;
        DebugMgr_LogInfo("FuSa: Fault cleared");
    }

    return cleared;
}

bool FuSa_GetFaultEntry(uint8_t index, FuSa_FaultEntry_t *entry) {
    if (index >= FUSA_FAULT_LOG_SIZE || entry == NULL) {
        return false;
    }

    memcpy(entry, &fault_log[index], sizeof(FuSa_FaultEntry_t));
    return true;
}

uint8_t FuSa_GetActiveFaultCount(void) {
    return active_fault_count;
}

void FuSa_AliveIndication(void) {
    alive_counter++;
    last_alive_time_ms = TIMER_GetTick();
}

bool FuSa_CheckRAMIntegrity(void) {
    /* Save original values */
    uint8_t original[FUSA_RAM_TEST_SIZE];
    for (uint8_t i = 0u; i < FUSA_RAM_TEST_SIZE; i++) {
        original[i] = ram_test_area[i];
    }

    /* Write pattern A (0x55) and verify */
    for (uint8_t i = 0u; i < FUSA_RAM_TEST_SIZE; i++) {
        ram_test_area[i] = FUSA_RAM_PATTERN_A;
    }
    for (uint8_t i = 0u; i < FUSA_RAM_TEST_SIZE; i++) {
        if (ram_test_area[i] != FUSA_RAM_PATTERN_A) {
            return false;
        }
    }

    /* Write pattern B (0xAA) and verify */
    for (uint8_t i = 0u; i < FUSA_RAM_TEST_SIZE; i++) {
        ram_test_area[i] = FUSA_RAM_PATTERN_B;
    }
    for (uint8_t i = 0u; i < FUSA_RAM_TEST_SIZE; i++) {
        if (ram_test_area[i] != FUSA_RAM_PATTERN_B) {
            return false;
        }
    }

    /* Restore original values */
    for (uint8_t i = 0u; i < FUSA_RAM_TEST_SIZE; i++) {
        ram_test_area[i] = original[i];
    }

    return true;
}

uint8_t FuSa_CalculateCRC8(const uint8_t *data, uint8_t length) {
    uint8_t crc = 0x00u;

    for (uint8_t i = 0u; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0u; bit < 8u; bit++) {
            if (crc & 0x80u) {
                crc = (crc << 1) ^ FUSA_CRC_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

void FuSa_EmergencyShutdown(void) {
    DebugMgr_LogError("FuSa: EMERGENCY SHUTDOWN!");

    current_state = FUSA_STATE_EMERGENCY;

    /* Open all contactors immediately */
    open_contactors();

    /* Disable all operations */
    contactor_allowed = false;
    charging_allowed = false;
    discharging_allowed = false;

    /* Enter infinite safe loop - system requires power cycle */
    while (1) {
        /* Toggle fault LED to indicate emergency */
        GPIO_ToggleBit(LED_RED_PORT, LED_RED_PIN);

        /* Simple delay loop */
        for (volatile uint32_t i = 0u; i < 500000u; i++) {
            __asm volatile ("nop");
        }
    }
}

/*===========================================================================*/
/* Private Function Implementations                                          */
/*===========================================================================*/

static void log_fault(FuSa_FaultType_t type, FuSa_Severity_t severity,
                      uint16_t data, uint8_t cell_idx) {
    /* Check if fault already logged and active */
    for (uint8_t i = 0u; i < FUSA_FAULT_LOG_SIZE; i++) {
        if (fault_log[i].fault_type == type && fault_log[i].active) {
            /* Fault already active, just update timestamp */
            fault_log[i].timestamp_ms = TIMER_GetTick();
            return;
        }
    }

    /* Log new fault */
    fault_log[fault_log_index].fault_type = type;
    fault_log[fault_log_index].severity = severity;
    fault_log[fault_log_index].timestamp_ms = TIMER_GetTick();
    fault_log[fault_log_index].fault_data = data;
    fault_log[fault_log_index].cell_index = cell_idx;
    fault_log[fault_log_index].active = true;

    fault_log_index = (fault_log_index + 1u) % FUSA_FAULT_LOG_SIZE;
    active_fault_count++;

    /* Log to debug output */
    char msg[64];
    snprintf(msg, sizeof(msg), "FuSa: Fault %d (sev=%d) cell=%d data=%u",
             type, severity, cell_idx, data);
    DebugMgr_LogWarning(msg);
}

static bool check_voltage_plausibility(void) {
    bool passed = true;

    for (uint8_t i = 0u; i < BMS_TOTAL_CELLS; i++) {
        uint16_t v = BMS_DB_GetCellVoltage(i);

        /* Check if voltage is within sensor range */
        if (v < BMS_FUSA_VOLT_PLAUSIBLE_MIN || v > BMS_FUSA_VOLT_PLAUSIBLE_MAX) {
            fault_debounce[FUSA_FAULT_VOLTAGE_PLAUSIBILITY]++;

            if (fault_debounce[FUSA_FAULT_VOLTAGE_PLAUSIBILITY] >= BMS_DIAG_OV_DEBOUNCE) {
                log_fault(FUSA_FAULT_VOLTAGE_PLAUSIBILITY, FUSA_SEVERITY_DEGRADED, v, i);
                passed = false;
            }
        } else {
            fault_debounce[FUSA_FAULT_VOLTAGE_PLAUSIBILITY] = 0u;
        }
    }

    return passed;
}

static bool check_temperature_plausibility(void) {
    bool passed = true;

    for (uint8_t i = 0u; i < BMS_TOTAL_CELLS; i++) {
        int8_t t = BMS_DB_GetCellTemperature(i);

        /* Check if temperature is within sensor range */
        if (t < BMS_FUSA_TEMP_PLAUSIBLE_MIN || t > BMS_FUSA_TEMP_PLAUSIBLE_MAX) {
            fault_debounce[FUSA_FAULT_TEMP_PLAUSIBILITY]++;

            if (fault_debounce[FUSA_FAULT_TEMP_PLAUSIBILITY] >= BMS_DIAG_OT_DEBOUNCE) {
                log_fault(FUSA_FAULT_TEMP_PLAUSIBILITY, FUSA_SEVERITY_DEGRADED,
                          (uint16_t)(t + 128), i);  /* Offset for positive value */
                passed = false;
            }
        } else {
            fault_debounce[FUSA_FAULT_TEMP_PLAUSIBILITY] = 0u;
        }
    }

    return passed;
}

static bool check_voltage_gradient(void) {
    bool passed = true;
    uint32_t time_delta = TIMER_GetTick() - prev_measurement.timestamp_ms;

    if (time_delta == 0u) {
        return true;  /* Avoid division by zero */
    }

    for (uint8_t i = 0u; i < BMS_TOTAL_CELLS; i++) {
        uint16_t v_current = BMS_DB_GetCellVoltage(i);
        uint16_t v_previous = prev_measurement.cell_voltages[i];

        /* Calculate absolute change */
        uint16_t delta_v;
        if (v_current > v_previous) {
            delta_v = v_current - v_previous;
        } else {
            delta_v = v_previous - v_current;
        }

        /* Normalize to 100ms period */
        uint32_t gradient = (delta_v * 100u) / time_delta;

        if (gradient > BMS_FUSA_VOLT_GRADIENT_MAX) {
            fault_debounce[FUSA_FAULT_VOLTAGE_GRADIENT]++;

            if (fault_debounce[FUSA_FAULT_VOLTAGE_GRADIENT] >= 2u) {
                log_fault(FUSA_FAULT_VOLTAGE_GRADIENT, FUSA_SEVERITY_WARNING,
                          (uint16_t)gradient, i);
                passed = false;
            }
        } else {
            fault_debounce[FUSA_FAULT_VOLTAGE_GRADIENT] = 0u;
        }
    }

    return passed;
}

static bool check_temperature_gradient(void) {
    bool passed = true;
    uint32_t time_delta = TIMER_GetTick() - prev_measurement.timestamp_ms;

    if (time_delta == 0u) {
        return true;
    }

    for (uint8_t i = 0u; i < BMS_TOTAL_CELLS; i++) {
        int8_t t_current = BMS_DB_GetCellTemperature(i);
        int8_t t_previous = prev_measurement.cell_temps[i];

        /* Calculate absolute change */
        int16_t delta_t = t_current - t_previous;
        if (delta_t < 0) {
            delta_t = -delta_t;
        }

        /* Normalize to 1 second period */
        uint32_t gradient = (delta_t * 1000u) / time_delta;

        if (gradient > (uint32_t)BMS_FUSA_TEMP_GRADIENT_MAX) {
            fault_debounce[FUSA_FAULT_TEMP_GRADIENT]++;

            if (fault_debounce[FUSA_FAULT_TEMP_GRADIENT] >= 2u) {
                log_fault(FUSA_FAULT_TEMP_GRADIENT, FUSA_SEVERITY_WARNING,
                          (uint16_t)gradient, i);
                passed = false;
            }
        } else {
            fault_debounce[FUSA_FAULT_TEMP_GRADIENT] = 0u;
        }
    }

    return passed;
}

static bool check_critical_limits(void) {
    bool passed = true;

    /* Check critical voltage limits */
    uint16_t max_v = BMS_DB_GetMaxCellVoltage();
    uint16_t min_v = BMS_DB_GetMinCellVoltage();

    if (max_v > BMS_CELL_VOLT_CRITICAL_OV) {
        log_fault(FUSA_FAULT_VOLTAGE_CRITICAL, FUSA_SEVERITY_CRITICAL, max_v, 0u);
        passed = false;
    }

    if (min_v < BMS_CELL_VOLT_CRITICAL_UV && min_v > 0u) {
        log_fault(FUSA_FAULT_VOLTAGE_CRITICAL, FUSA_SEVERITY_CRITICAL, min_v, 0u);
        passed = false;
    }

    /* Check critical temperature limit */
    int8_t max_t = BMS_DB_GetMaxTemperature();

    if (max_t > BMS_TEMP_CRITICAL_HIGH) {
        log_fault(FUSA_FAULT_TEMP_CRITICAL, FUSA_SEVERITY_CRITICAL,
                  (uint16_t)(max_t + 128), 0u);
        passed = false;
    }

    if (max_t < BMS_TEMP_CRITICAL_LOW) {
        log_fault(FUSA_FAULT_TEMP_CRITICAL, FUSA_SEVERITY_CRITICAL,
                  (uint16_t)(max_t + 128), 0u);
        passed = false;
    }

    return passed;
}

static bool check_data_freshness(void) {
    const BMS_CellData_t *data = BMS_DB_GetAllData();

    if (data == NULL || !data->data_valid) {
        fault_debounce[FUSA_FAULT_DATA_STALE]++;

        if (fault_debounce[FUSA_FAULT_DATA_STALE] >= 3u) {
            log_fault(FUSA_FAULT_DATA_STALE, FUSA_SEVERITY_DEGRADED, 0u, 0u);
            return false;
        }
    }

    uint32_t data_age = TIMER_GetTick() - data->last_update_ms;

    if (data_age > BMS_DATA_STALE_TIMEOUT_MS) {
        fault_debounce[FUSA_FAULT_DATA_STALE]++;

        if (fault_debounce[FUSA_FAULT_DATA_STALE] >= 3u) {
            log_fault(FUSA_FAULT_DATA_STALE, FUSA_SEVERITY_DEGRADED,
                      (uint16_t)data_age, 0u);
            return false;
        }
    } else {
        fault_debounce[FUSA_FAULT_DATA_STALE] = 0u;
    }

    return true;
}

static bool check_alive_supervision(void) {
    uint32_t time_since_alive = TIMER_GetTick() - last_alive_time_ms;

    if (time_since_alive > (BMS_FUSA_ALIVE_TIMEOUT * BMS_MEASUREMENT_PERIOD_MS)) {
        fault_debounce[FUSA_FAULT_ALIVE_TIMEOUT]++;

        if (fault_debounce[FUSA_FAULT_ALIVE_TIMEOUT] >= 2u) {
            log_fault(FUSA_FAULT_ALIVE_TIMEOUT, FUSA_SEVERITY_DEGRADED,
                      (uint16_t)time_since_alive, 0u);
            return false;
        }
    } else {
        fault_debounce[FUSA_FAULT_ALIVE_TIMEOUT] = 0u;
    }

    return true;
}

static void update_state_machine(void) {
    FuSa_Severity_t max_severity = FUSA_SEVERITY_INFO;

    /* Find highest severity active fault */
    for (uint8_t i = 0u; i < FUSA_FAULT_LOG_SIZE; i++) {
        if (fault_log[i].active && fault_log[i].severity > max_severity) {
            max_severity = fault_log[i].severity;
        }
    }

    /* Determine state based on maximum severity */
    switch (max_severity) {
        case FUSA_SEVERITY_INFO:
        case FUSA_SEVERITY_WARNING:
            if (current_state != FUSA_STATE_EMERGENCY) {
                current_state = FUSA_STATE_NORMAL;
            }
            break;

        case FUSA_SEVERITY_DEGRADED:
            if (current_state != FUSA_STATE_SAFE &&
                current_state != FUSA_STATE_EMERGENCY) {
                current_state = FUSA_STATE_DEGRADED;
                DebugMgr_LogWarning("FuSa: Entering DEGRADED state");
            }
            break;

        case FUSA_SEVERITY_CRITICAL:
            enter_safe_state();
            break;
    }
}

static void enter_safe_state(void) {
    if (current_state == FUSA_STATE_EMERGENCY) {
        return;  /* Already in emergency */
    }

    DebugMgr_LogError("FuSa: Entering SAFE state!");
    current_state = FUSA_STATE_SAFE;

    /* Open contactors */
    open_contactors();

    /* Disable operations */
    contactor_allowed = false;
    charging_allowed = false;
    discharging_allowed = false;
}

static void open_contactors(void) {
    /* Open main contactor (active low logic typical) */
    GPIO_ClearBit(CONTACTOR_MAIN_PORT, CONTACTOR_MAIN_PIN);

    /* Open precharge contactor */
    GPIO_ClearBit(CONTACTOR_PRECHARGE_PORT, CONTACTOR_PRECHARGE_PIN);

    /* Assert safety shutdown output */
    GPIO_SetBit(SAFETY_SHUTDOWN_PORT, SAFETY_SHUTDOWN_PIN);

    DebugMgr_LogInfo("FuSa: Contactors opened");
}

static void update_operation_permissions(void) {
    switch (current_state) {
        case FUSA_STATE_NORMAL:
            /* Full operation allowed */
            contactor_allowed = true;

            /* Check temperature limits for charge/discharge */
            {
                int8_t max_t = BMS_DB_GetMaxTemperature();
                uint16_t max_v = BMS_DB_GetMaxCellVoltage();
                uint16_t min_v = BMS_DB_GetMinCellVoltage();

                charging_allowed = (max_t < BMS_TEMP_MAX_CHARGE) &&
                                   (max_t > BMS_TEMP_MIN_CHARGE) &&
                                   (max_v < BMS_CELL_VOLT_MAX);

                discharging_allowed = (max_t < BMS_TEMP_MAX_DISCHARGE) &&
                                      (max_t > BMS_TEMP_MIN_DISCHARGE) &&
                                      (min_v > BMS_CELL_VOLT_MIN);
            }
            break;

        case FUSA_STATE_DEGRADED:
            /* Limited operation */
            contactor_allowed = true;
            charging_allowed = false;  /* Disable charging in degraded mode */
            discharging_allowed = true; /* Allow discharge to empty pack safely */
            break;

        case FUSA_STATE_SAFE:
        case FUSA_STATE_EMERGENCY:
        case FUSA_STATE_INIT:
        default:
            /* No operation allowed */
            contactor_allowed = false;
            charging_allowed = false;
            discharging_allowed = false;
            break;
    }
}

static FuSa_Severity_t get_fault_severity(FuSa_FaultType_t type) {
    switch (type) {
        case FUSA_FAULT_VOLTAGE_CRITICAL:
        case FUSA_FAULT_TEMP_CRITICAL:
        case FUSA_FAULT_RAM_CORRUPTION:
            return FUSA_SEVERITY_CRITICAL;

        case FUSA_FAULT_VOLTAGE_PLAUSIBILITY:
        case FUSA_FAULT_TEMP_PLAUSIBILITY:
        case FUSA_FAULT_COMM_LOSS:
        case FUSA_FAULT_DATA_STALE:
        case FUSA_FAULT_ALIVE_TIMEOUT:
        case FUSA_FAULT_CONTACTOR_FEEDBACK:
            return FUSA_SEVERITY_DEGRADED;

        case FUSA_FAULT_VOLTAGE_GRADIENT:
        case FUSA_FAULT_TEMP_GRADIENT:
        case FUSA_FAULT_REDUNDANCY_MISMATCH:
            return FUSA_SEVERITY_WARNING;

        default:
            return FUSA_SEVERITY_INFO;
    }
}
