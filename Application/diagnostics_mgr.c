#include "diagnostics_mgr.h"
#include "bms_database.h"
#include "bms_config.h"
#include "debug_info_mgr.h"

static bool fault_active[DIAG_FAULT_COUNT];
static uint8_t fault_counter[DIAG_FAULT_COUNT];

void Diagnostics_Init(void) {
    for (uint8_t i = 0; i < DIAG_FAULT_COUNT; i++) {
        fault_active[i] = false;
        fault_counter[i] = 0;
    }
    DebugMgr_LogInfo("Diagnostics initialized");
}

void Diagnostics_CheckVoltages(void) {
    uint16_t max_v = BMS_DB_GetMaxCellVoltage();
    uint16_t min_v = BMS_DB_GetMinCellVoltage();
    
    /* Check overvoltage */
    if (max_v > BMS_CELL_VOLT_MAX) {
        if (!fault_active[DIAG_FAULT_OVERVOLTAGE]) {
            DebugMgr_LogError("Overvoltage detected");
            fault_active[DIAG_FAULT_OVERVOLTAGE] = true;
        }
        fault_counter[DIAG_FAULT_OVERVOLTAGE]++;
    } else {
        fault_active[DIAG_FAULT_OVERVOLTAGE] = false;
    }
    
    /* Check undervoltage */
    if (min_v < BMS_CELL_VOLT_MIN && min_v > 0) {
        if (!fault_active[DIAG_FAULT_UNDERVOLTAGE]) {
            DebugMgr_LogError("Undervoltage detected");
            fault_active[DIAG_FAULT_UNDERVOLTAGE] = true;
        }
        fault_counter[DIAG_FAULT_UNDERVOLTAGE]++;
    } else {
        fault_active[DIAG_FAULT_UNDERVOLTAGE] = false;
    }
}

void Diagnostics_CheckTemperatures(void) {
    int8_t max_temp = BMS_DB_GetMaxTemperature();
    
    if (max_temp > BMS_TEMP_MAX_DISCHARGE) {
        if (!fault_active[DIAG_FAULT_OVERTEMP]) {
            DebugMgr_LogError("Overtemperature detected");
            fault_active[DIAG_FAULT_OVERTEMP] = true;
        }
        fault_counter[DIAG_FAULT_OVERTEMP]++;
    } else {
        fault_active[DIAG_FAULT_OVERTEMP] = false;
    }
}

uint8_t Diagnostics_GetFaultCount(void) {
    uint8_t count = 0;
    for (uint8_t i = 0; i < DIAG_FAULT_COUNT; i++) {
        if (fault_active[i]) count++;
    }
    return count;
}

bool Diagnostics_IsFaultActive(DiagFaultType_t fault) {
    if (fault >= DIAG_FAULT_COUNT) return false;
    return fault_active[fault];
}