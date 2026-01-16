#include "debug_info_mgr.h"
#include "uart.h"
#include "timer.h"
#include <string.h>
#include <stdio.h>

void DebugMgr_Init(void) {
    UART_Config_t uart_config = {
        .baudrate = 115200,
        .data_bits = 8,
        .parity = 0,
        .stop_bits = 1
    };
    UART_Init(&uart_config);
}

static void log_with_level(const char *level, const char *msg) {
    char buffer[128];
    uint32_t tick = TIMER_GetTick();
    snprintf(buffer, sizeof(buffer), "[%lu][%s] %s\r\n", tick, level, msg);
    UART_TransmitString(buffer);
}

void DebugMgr_LogInfo(const char *msg) {
    log_with_level("INFO", msg);
}

void DebugMgr_LogWarning(const char *msg) {
    log_with_level("WARN", msg);
}

void DebugMgr_LogError(const char *msg) {
    log_with_level("ERROR", msg);
}

void DebugMgr_LogData(const char *label, uint32_t value) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%s: %lu", label, value);
    log_with_level("DATA", buffer);
}