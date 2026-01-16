#include "bms_database.h"
#include "timer.h"
#include <string.h>

static BMS_CellData_t cell_data;
static BMS_SystemStatus_t system_status;

void BMS_DB_Init(void) {
    memset(&cell_data, 0, sizeof(cell_data));
    memset(&system_status, 0, sizeof(system_status));
    system_status.system_ok = true;
}

void BMS_DB_UpdateCellVoltages(uint8_t slave_id, const uint16_t *voltages) {
    if (slave_id >= BMS_NUM_SLAVES || voltages == NULL) return;
    
    uint8_t offset = slave_id * BMS_CELLS_PER_SLAVE;
    for (uint8_t i = 0; i < BMS_CELLS_PER_SLAVE; i++) {
        cell_data.cell_voltages_mV[offset + i] = voltages[i];
    }
    cell_data.last_update_ms = TIMER_GetTick();
    cell_data.data_valid = true;
}

void BMS_DB_UpdateTemperatures(uint8_t slave_id, const int8_t *temps) {
    if (slave_id >= BMS_NUM_SLAVES || temps == NULL) return;
    
    uint8_t offset = slave_id * BMS_CELLS_PER_SLAVE;
    for (uint8_t i = 0; i < BMS_CELLS_PER_SLAVE; i++) {
        cell_data.cell_temperatures_C[offset + i] = temps[i];
    }
}

uint16_t BMS_DB_GetCellVoltage(uint8_t cell_index) {
    if (cell_index >= BMS_TOTAL_CELLS) return 0;
    return cell_data.cell_voltages_mV[cell_index];
}

int8_t BMS_DB_GetCellTemperature(uint8_t cell_index) {
    if (cell_index >= BMS_TOTAL_CELLS) return 0;
    return cell_data.cell_temperatures_C[cell_index];
}

uint16_t BMS_DB_GetMaxCellVoltage(void) {
    uint16_t max = cell_data.cell_voltages_mV[0];
    for (uint8_t i = 1; i < BMS_TOTAL_CELLS; i++) {
        if (cell_data.cell_voltages_mV[i] > max) {
            max = cell_data.cell_voltages_mV[i];
        }
    }
    return max;
}

uint16_t BMS_DB_GetMinCellVoltage(void) {
    uint16_t min = cell_data.cell_voltages_mV[0];
    for (uint8_t i = 1; i < BMS_TOTAL_CELLS; i++) {
        if (cell_data.cell_voltages_mV[i] < min) {
            min = cell_data.cell_voltages_mV[i];
        }
    }
    return min;
}

int8_t BMS_DB_GetMaxTemperature(void) {
    int8_t max = cell_data.cell_temperatures_C[0];
    for (uint8_t i = 1; i < BMS_TOTAL_CELLS; i++) {
        if (cell_data.cell_temperatures_C[i] > max) {
            max = cell_data.cell_temperatures_C[i];
        }
    }
    return max;
}

void BMS_DB_SetSystemStatus(const BMS_SystemStatus_t *status) {
    if (status != NULL) {
        memcpy(&system_status, status, sizeof(BMS_SystemStatus_t));
    }
}

void BMS_DB_GetSystemStatus(BMS_SystemStatus_t *status) {
    if (status != NULL) {
        memcpy(status, &system_status, sizeof(BMS_SystemStatus_t));
    }
}

const BMS_CellData_t* BMS_DB_GetAllData(void) {
    return &cell_data;
}