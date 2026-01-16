/**
 * @file fan_control.h
 * @brief Fan Control - Application Layer
 */

#ifndef APP_FAN_CONTROL_H
#define APP_FAN_CONTROL_H

#include <stdint.h>

void FanControl_Init(void);
void FanControl_Update(int8_t temperature_C);
void FanControl_SetSpeed(uint8_t speed_percent);
uint8_t FanControl_GetSpeed(void);

#endif