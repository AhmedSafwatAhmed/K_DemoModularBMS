#include "thermal_mgr.h"
#include "bms_database.h"
#include "fan_control.h"

void ThermalMgr_Init(void) {
    /* Initialize thermal management */
}

void ThermalMgr_Update(void) {
    int8_t max_temp = BMS_DB_GetMaxTemperature();
    
    /* Update fan control based on temperature */
    FanControl_Update(max_temp);
}

int8_t ThermalMgr_GetMaxTemp(void) {
    return BMS_DB_GetMaxTemperature();
}