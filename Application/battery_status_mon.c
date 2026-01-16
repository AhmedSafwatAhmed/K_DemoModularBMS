#include "battery_status_mon.h"
#include "bms_database.h"
#include "bms_config.h"

static uint8_t soc_percent = 50;  /* Initial 50% SOC */

void BatteryStatus_Init(void) {
    /* Initialize SOC estimation */
}

void BatteryStatus_Update(void) {
    /* Simplified SOC estimation based on voltage */
    uint16_t avg_v = 0;
    for (uint8_t i = 0; i < BMS_TOTAL_CELLS; i++) {
        avg_v += BMS_DB_GetCellVoltage(i);
    }
    avg_v /= BMS_TOTAL_CELLS;
    
    /* Linear mapping: 2800mV=0%, 4200mV=100% */
    if (avg_v <= BMS_CELL_VOLT_MIN) {
        soc_percent = 0;
    } else if (avg_v >= BMS_CELL_VOLT_MAX) {
        soc_percent = 100;
    } else {
        soc_percent = ((avg_v - BMS_CELL_VOLT_MIN) * 100) / 
                      (BMS_CELL_VOLT_MAX - BMS_CELL_VOLT_MIN);
    }
}

uint8_t BatteryStatus_GetSOC(void) {
    return soc_percent;
}

uint32_t BatteryStatus_GetPackVoltage(void) {
    uint32_t pack_v = 0;
    for (uint8_t i = 0; i < BMS_TOTAL_CELLS; i++) {
        pack_v += BMS_DB_GetCellVoltage(i);
    }
    return pack_v;  /* Total pack voltage in mV */
}