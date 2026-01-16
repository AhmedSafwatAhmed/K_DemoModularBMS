#include "fusa_supervisor.h"
#include "bms_database.h"
#include "bms_config.h"
#include "debug_info_mgr.h"

static bool safety_ok = true;

void FuSa_Init(void) {
    DebugMgr_LogInfo("FuSa Supervisor initialized");
}

bool FuSa_PerformSafetyChecks(void) {
    safety_ok = true;
    
    /* Plausibility check: voltages in valid range */
    for (uint8_t i = 0; i < BMS_TOTAL_CELLS; i++) {
        uint16_t v = BMS_DB_GetCellVoltage(i);
        if (v < 2000 || v > 5000) {  /* Outside sensor range */
            DebugMgr_LogWarning("Implausible voltage reading");
            safety_ok = false;
        }
    }
    
    /* Check temperature plausibility */
    for (uint8_t i = 0; i < BMS_TOTAL_CELLS; i++) {
        int8_t t = BMS_DB_GetCellTemperature(i);
        if (t < -40 || t > 125) {
            DebugMgr_LogWarning("Implausible temperature reading");
            safety_ok = false;
        }
    }
    
    return safety_ok;
}

bool FuSa_GetSafetyStatus(void) {
    return safety_ok;
}