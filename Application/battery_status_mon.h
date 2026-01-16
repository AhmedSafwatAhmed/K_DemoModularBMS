/**
 * @file battery_status_mon.h
 * @brief Battery Status Monitor - Application Layer
 */

#ifndef APP_BATTERY_STATUS_MON_H
#define APP_BATTERY_STATUS_MON_H

#include <stdint.h>

void BatteryStatus_Init(void);
void BatteryStatus_Update(void);
uint8_t BatteryStatus_GetSOC(void);
uint32_t BatteryStatus_GetPackVoltage(void);

#endif