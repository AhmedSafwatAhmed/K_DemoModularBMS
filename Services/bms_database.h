/**
 * @file bms_database.h
 * @brief BMS Central Database - Service Layer
 */

#ifndef SVC_BMS_DATABASE_H
#define SVC_BMS_DATABASE_H

#include <stdint.h>
#include <stdbool.h>
#include "bms_config.h"

typedef struct {
    uint16_t cell_voltages_mV[BMS_TOTAL_CELLS];
    int8_t   cell_temperatures_C[BMS_TOTAL_CELLS];
    uint32_t last_update_ms;
    bool     data_valid;
} BMS_CellData_t;

typedef struct {
    bool system_ok;
    bool overvoltage_fault;
    bool undervoltage_fault;
    bool overtemp_fault;
    bool comm_fault;
    uint8_t active_fault_count;
} BMS_SystemStatus_t;

void BMS_DB_Init(void);
void BMS_DB_UpdateCellVoltages(uint8_t slave_id, const uint16_t *voltages);
void BMS_DB_UpdateTemperatures(uint8_t slave_id, const int8_t *temps);
uint16_t BMS_DB_GetCellVoltage(uint8_t cell_index);
int8_t BMS_DB_GetCellTemperature(uint8_t cell_index);
uint16_t BMS_DB_GetMaxCellVoltage(void);
uint16_t BMS_DB_GetMinCellVoltage(void);
int8_t BMS_DB_GetMaxTemperature(void);
void BMS_DB_SetSystemStatus(const BMS_SystemStatus_t *status);
void BMS_DB_GetSystemStatus(BMS_SystemStatus_t *status);
const BMS_CellData_t* BMS_DB_GetAllData(void);

#endif