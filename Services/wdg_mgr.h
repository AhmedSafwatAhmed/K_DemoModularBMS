/**
 * @file wdg_mgr.h
 * @brief Watchdog Manager - Service Layer
 */

#ifndef SVC_WDG_MGR_H
#define SVC_WDG_MGR_H

#include <stdint.h>
#include <stdbool.h>

void WDG_Init(uint32_t timeout_ms);
void WDG_Refresh(void);
void WDG_RegisterTask(const char *task_name);

#endif