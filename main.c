/**
 * @file main.c
 * @brief Main BMS Application Entry Point
 * @author EV-BMS Development Team
 * @date 2025
 * 
 * This is the main application file that initializes all system components
 * and runs the main BMS control loop.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* Configuration */
#include "bms_config.h"
#include "board_config.h"

/* MCAL Layer */
#include "spi_mc33664b.h"      
#include "uart_kl25z.h"        
#include "pwm.h"
#include "gpio.h"
#include "timer.h"
#include "i2c.h"

/* HAL Layer */
#include "mc33771_driver.h"
#include "lcd_if.h"

/* Service Layer */
#include "debug_info_mgr.h"
#include "bms_database.h"
#include "wdg_mgr.h"

/* Application Layer */
#include "diagnostics_mgr.h"
#include "fusa_supervisor.h"
#include "battery_status_mon.h"
#include "fan_control.h"
#include "thermal_mgr.h"
#include "cell_balancing_mgr.h"

/*===========================================================================*/
/* Global Variables                                                          */
/*===========================================================================*/
static uint32_t last_measurement_time = 0;
static uint32_t last_heartbeat_time = 0;

/*===========================================================================*/
/* Function Prototypes                                                       */
/*===========================================================================*/
static bool BMS_InitializeSystem(void);
static void BMS_MeasurementCycle(void);
static void BMS_HeartbeatTask(void);
static void BMS_UpdateLCD(void);

/*===========================================================================*/
/* Main Function                                                             */
/*===========================================================================*/
int main(void) {
    /* System initialization */
    if (!BMS_InitializeSystem()) {
        /* Initialization failed - enter safe state */
        while (1) {
            /* Blink error LED */
            GPIO_ToggleBit(1, LED_RED_PIN);  /* Port B, Red LED */
            TIMER_DelayMs(500);
        }
    }
    
    DebugMgr_LogInfo("=================================");
    DebugMgr_LogInfo(" EV-BMS System Started");
    DebugMgr_LogInfo("=================================");
    
    /* Main control loop */
    while (1) {
        uint32_t current_time = TIMER_GetTick();
        
        /* Periodic measurement cycle */
        if ((current_time - last_measurement_time) >= BMS_MEASUREMENT_PERIOD_MS) {
            BMS_MeasurementCycle();
            last_measurement_time = current_time;
            
            /* Heartbeat LED */
            GPIO_ToggleBit(1, LED_GREEN_PIN);  /* Port B, Green LED */
        }
        
        /* Periodic heartbeat/logging task */
        if ((current_time - last_heartbeat_time) >= BMS_HEARTBEAT_PERIOD_MS) {
            BMS_HeartbeatTask();
            last_heartbeat_time = current_time;
        }
        
        /* Refresh watchdog */
        #if BMS_ENABLE_WATCHDOG
        WDG_Refresh();
        #endif
        
        /* Small delay to prevent busy-waiting */
        TIMER_DelayMs(10);
    }
    
    return 0;
}

/*===========================================================================*/
/* Static Function Implementations                                           */
/*===========================================================================*/

/**
 * @brief Initialize all BMS subsystems
 * @return true on success, false on failure
 */
static bool BMS_InitializeSystem(void) {
    /* Initialize system timer first (needed by other modules) */
    if (TIMER_Init() != TIMER_OK) {
        return false;
    }
    
    /* Initialize GPIO for status LEDs */
    GPIO_Init(1, LED_RED_PIN, GPIO_DIR_OUTPUT);    /* Red LED */
    GPIO_Init(1, LED_GREEN_PIN, GPIO_DIR_OUTPUT);  /* Green LED */
    GPIO_Init(3, LED_BLUE_PIN, GPIO_DIR_OUTPUT);   /* Blue LED */
    
    /* Turn on blue LED during initialization */
    GPIO_SetBit(3, LED_BLUE_PIN);
    
    /* Initialize Service Layer */
    DebugMgr_Init();
    BMS_DB_Init();
    
    #if BMS_ENABLE_WATCHDOG
    WDG_Init(BMS_WATCHDOG_TIMEOUT_MS);
    #endif
    
    DebugMgr_LogInfo("Initializing BMS subsystems...");
    
    /* Initialize HAL Layer */
    if (!SlaveIF_Init()) {
        DebugMgr_LogError("Slave interface init failed");
        return false;
    }
    
    #if BMS_ENABLE_LCD
    if (!LCD_Init()) {
        DebugMgr_LogWarning("LCD init failed (non-critical)");
    } else {
        LCD_Clear();
        LCD_PrintAt(0, 0, "EV-BMS v1.0");
        LCD_PrintAt(1, 0, "Initializing...");
    }
    #endif
    
    /* Initialize Application Layer */
    Diagnostics_Init();
    
    #if BMS_ENABLE_FUSA
    FuSa_Init();
    #endif
    
    BatteryStatus_Init();
    FanControl_Init();
    ThermalMgr_Init();
    
    #if BMS_ENABLE_BALANCING
    CellBalance_Init();
    #endif
    
    /* Brief delay for stabilization */
    TIMER_DelayMs(100);
    
    /* Turn off blue LED, turn on green LED */
    GPIO_ClearBit(3, LED_BLUE_PIN);
    GPIO_SetBit(1, LED_GREEN_PIN);
    
    DebugMgr_LogInfo("BMS initialization complete");
    
    return true;
}

/**
 * @brief Execute one measurement cycle
 */
static void BMS_MeasurementCycle(void) {
    SlaveData_t slave_data;
    
    /* Request measurements from all slaves */
    for (uint8_t slave_id = 0; slave_id < BMS_NUM_SLAVES; slave_id++) {
        /* Request measurement */
        if (!SlaveIF_RequestMeasurements(slave_id)) {
            DebugMgr_LogError("Failed to request measurements");
            continue;
        }
        
        /* Small delay for slave processing */
        TIMER_DelayMs(10);
        
        /* Retrieve measurement data */
        if (SlaveIF_GetSlaveData(slave_id, &slave_data)) {
            /* Update database with new measurements */
            BMS_DB_UpdateCellVoltages(slave_id, slave_data.cell_voltages_mV);
            
            /* Simulated temperature data (in production, read from slaves) */
            int8_t temps[BMS_CELLS_PER_SLAVE];
            for (uint8_t i = 0; i < BMS_CELLS_PER_SLAVE; i++) {
                temps[i] = 35 + (i % 3);  /* Simulated: 35-37°C */
            }
            BMS_DB_UpdateTemperatures(slave_id, temps);
        }
    }
    
    /* Run diagnostic checks */
    Diagnostics_CheckVoltages();
    Diagnostics_CheckTemperatures();
    
    /* Perform safety checks */
    #if BMS_ENABLE_FUSA
    FuSa_PerformSafetyChecks();
    #endif
    
    /* Update battery status */
    BatteryStatus_Update();
    
    /* Update thermal management */
    ThermalMgr_Update();
    
    /* Update cell balancing */
    #if BMS_ENABLE_BALANCING
    CellBalance_Update();
    #endif
    
    /* Update LCD display */
    #if BMS_ENABLE_LCD
    BMS_UpdateLCD();
    #endif
}

/**
 * @brief Periodic heartbeat task - logging and status reporting
 */
static void BMS_HeartbeatTask(void) {
    char buffer[128];
    
    DebugMgr_LogInfo("--- System Heartbeat ---");
    
    /* Log all cell voltages */
    for (uint8_t i = 0; i < BMS_TOTAL_CELLS; i++) {
        uint16_t voltage = BMS_DB_GetCellVoltage(i);
        snprintf(buffer, sizeof(buffer), "Cell[%d]: %d mV", i, voltage);
        DebugMgr_LogData(buffer, voltage);
    }
    
    /* Log min/max voltages */
    uint16_t min_v = BMS_DB_GetMinCellVoltage();
    uint16_t max_v = BMS_DB_GetMaxCellVoltage();
    snprintf(buffer, sizeof(buffer), "Voltage Range: %d - %d mV (Δ=%d mV)", 
             min_v, max_v, max_v - min_v);
    DebugMgr_LogInfo(buffer);
    
    /* Log temperatures */
    int8_t max_temp = BMS_DB_GetMaxTemperature();
    snprintf(buffer, sizeof(buffer), "Max Temperature: %d°C", max_temp);
    DebugMgr_LogData(buffer, max_temp);
    
    /* Log fan speed */
    uint8_t fan_speed = FanControl_GetSpeed();
    snprintf(buffer, sizeof(buffer), "Fan Speed: %d%%", fan_speed);
    DebugMgr_LogData(buffer, fan_speed);
    
    /* Log SOC */
    uint8_t soc = BatteryStatus_GetSOC();
    snprintf(buffer, sizeof(buffer), "State of Charge: %d%%", soc);
    DebugMgr_LogData(buffer, soc);
    
    /* Log pack voltage */
    uint32_t pack_v = BatteryStatus_GetPackVoltage();
    snprintf(buffer, sizeof(buffer), "Pack Voltage: %lu mV", pack_v);
    DebugMgr_LogData(buffer, (uint32_t)pack_v);
    
    /* Log fault status */
    uint8_t fault_count = Diagnostics_GetFaultCount();
    if (fault_count > 0) {
        snprintf(buffer, sizeof(buffer), "Active Faults: %d", fault_count);
        DebugMgr_LogWarning(buffer);
        
        /* Turn on red LED if faults present */
        GPIO_SetBit(1, LED_RED_PIN);
    } else {
        DebugMgr_LogInfo("No active faults");
        GPIO_ClearBit(1, LED_RED_PIN);
    }
    
    DebugMgr_LogInfo("------------------------");
}

/**
 * @brief Update LCD display with current status
 */
static void BMS_UpdateLCD(void) {
    char line1[17];
    char line2[17];
    
    uint16_t max_v = BMS_DB_GetMaxCellVoltage();
    int8_t max_temp = BMS_DB_GetMaxTemperature();
    uint8_t soc = BatteryStatus_GetSOC();
    
    /* Line 1: SOC and Max Voltage */
    snprintf(line1, sizeof(line1), "SOC:%3d%% %4dmV", soc, max_v);
    
    /* Line 2: Temperature and Fan */
    snprintf(line2, sizeof(line2), "T:%3dC Fan:%3d%%", 
             max_temp, FanControl_GetSpeed());
    
    LCD_PrintAt(0, 0, line1);
    LCD_PrintAt(1, 0, line2);
}