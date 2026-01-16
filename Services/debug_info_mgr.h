/**
 * @file debug_info_mgr.h
 * @brief Debug Information Manager - Service Layer
 */

#ifndef SVC_DEBUG_INFO_MGR_H
#define SVC_DEBUG_INFO_MGR_H

#include <stdint.h>

typedef enum {
    DEBUG_LEVEL_INFO,
    DEBUG_LEVEL_WARNING,
    DEBUG_LEVEL_ERROR,
    DEBUG_LEVEL_DATA
} DebugLevel_t;

void DebugMgr_Init(void);
void DebugMgr_LogInfo(const char *msg);
void DebugMgr_LogWarning(const char *msg);
void DebugMgr_LogError(const char *msg);
void DebugMgr_LogData(const char *label, uint32_t value);

#endif