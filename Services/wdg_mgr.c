#include "wdg_mgr.h"
#include "debug_info_mgr.h"

/* Simplified watchdog - production would use hardware WDG */
static uint32_t wdg_timeout_ms = 1000;
static uint32_t last_refresh_tick = 0;

void WDG_Init(uint32_t timeout_ms) {
    wdg_timeout_ms = timeout_ms;
    DebugMgr_LogInfo("Watchdog initialized");
}

void WDG_Refresh(void) {
    /* In production: kick hardware watchdog */
    last_refresh_tick = 0; /* TIMER_GetTick() */
}

void WDG_RegisterTask(const char *task_name) {
    /* In production: register task for supervision */
}