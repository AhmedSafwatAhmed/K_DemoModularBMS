/**
 * @file diagnostics_mgr.h
 * @brief Diagnostics Manager - Application Layer
 */

#ifndef APP_DIAGNOSTICS_MGR_H
#define APP_DIAGNOSTICS_MGR_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    DIAG_FAULT_OVERVOLTAGE,
    DIAG_FAULT_UNDERVOLTAGE,
    DIAG_FAULT_OVERTEMP,
    DIAG_FAULT_COMM_ERROR,
    DIAG_FAULT_COUNT
} DiagFaultType_t;

void Diagnostics_Init(void);
void Diagnostics_CheckVoltages(void);
void Diagnostics_CheckTemperatures(void);
uint8_t Diagnostics_GetFaultCount(void);
bool Diagnostics_IsFaultActive(DiagFaultType_t fault);

#endif