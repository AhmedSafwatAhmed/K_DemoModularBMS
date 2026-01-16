/**
 * @file thermal_mgr.h
 * @brief Thermal Manager - Application Layer
 */

#ifndef APP_THERMAL_MGR_H
#define APP_THERMAL_MGR_H

#include <stdint.h>

void ThermalMgr_Init(void);
void ThermalMgr_Update(void);
int8_t ThermalMgr_GetMaxTemp(void);

#endif