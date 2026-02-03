/**
 * @file bms_config.h
 * @brief BMS System Configuration Parameters
 * @details Central configuration file for the EV Battery Management System
 *          Modify these parameters according to your battery pack specifications
 */

#ifndef BMS_CONFIG_H
#define BMS_CONFIG_H

/*===========================================================================*/
/* Battery Pack Configuration                                                */
/*===========================================================================*/

/** Number of MC33771 slave ICs in the daisy chain */
#define BMS_NUM_SLAVES              2u

/** Number of cells monitored per slave (MC33771 supports up to 14) */
#define BMS_CELLS_PER_SLAVE         4u

/** Total number of cells in the pack */
#define BMS_TOTAL_CELLS             (BMS_NUM_SLAVES * BMS_CELLS_PER_SLAVE)

/** Number of temperature sensors per slave */
#define BMS_TEMPS_PER_SLAVE         2u

/** Total number of temperature sensors */
#define BMS_TOTAL_TEMP_SENSORS      (BMS_NUM_SLAVES * BMS_TEMPS_PER_SLAVE)

/*===========================================================================*/
/* Cell Voltage Limits (millivolts)                                          */
/*===========================================================================*/

/** Minimum cell voltage (fully discharged) */
#define BMS_CELL_VOLT_MIN           2800u

/** Maximum cell voltage (fully charged) */
#define BMS_CELL_VOLT_MAX           4200u

/** Nominal cell voltage */
#define BMS_CELL_VOLT_NOMINAL       3700u

/** Overvoltage threshold for fault detection */
#define BMS_CELL_VOLT_OV_THRESH     4250u

/** Undervoltage threshold for fault detection */
#define BMS_CELL_VOLT_UV_THRESH     2700u

/** Voltage difference to trigger cell balancing */
#define BMS_CELL_VOLT_BALANCE_DIFF  50u

/** Balance target threshold - stop balancing when within this of min cell */
#define BMS_CELL_VOLT_BALANCE_TARGET 10u

/** Critical overvoltage - immediate shutdown */
#define BMS_CELL_VOLT_CRITICAL_OV   4300u

/** Critical undervoltage - immediate shutdown */
#define BMS_CELL_VOLT_CRITICAL_UV   2500u

/*===========================================================================*/
/* Temperature Limits (degrees Celsius)                                      */
/*===========================================================================*/

/** Maximum temperature for charging */
#define BMS_TEMP_MAX_CHARGE         45

/** Maximum temperature for discharging */
#define BMS_TEMP_MAX_DISCHARGE      55

/** Minimum temperature for charging */
#define BMS_TEMP_MIN_CHARGE         0

/** Minimum temperature for discharging */
#define BMS_TEMP_MIN_DISCHARGE      (-20)

/** Critical high temperature - immediate shutdown */
#define BMS_TEMP_CRITICAL_HIGH      60

/** Critical low temperature - immediate shutdown */
#define BMS_TEMP_CRITICAL_LOW       (-30)

/** Temperature threshold to turn fan ON */
#define BMS_TEMP_FAN_ON             40

/** Temperature threshold to turn fan OFF (hysteresis) */
#define BMS_TEMP_FAN_OFF            35

/*===========================================================================*/
/* Timing Configuration (milliseconds)                                       */
/*===========================================================================*/

/** Main measurement cycle period */
#define BMS_MEASUREMENT_PERIOD_MS   100u

/** Heartbeat/logging period */
#define BMS_HEARTBEAT_PERIOD_MS     1000u

/** Cell balancing update period */
#define BMS_BALANCING_PERIOD_MS     500u

/** Watchdog timeout period */
#define BMS_WATCHDOG_TIMEOUT_MS     200u

/** Communication timeout for slave response */
#define BMS_COMM_TIMEOUT_MS         50u

/** Data freshness timeout - stale data if older than this */
#define BMS_DATA_STALE_TIMEOUT_MS   500u

/*===========================================================================*/
/* Fan/PWM Configuration                                                     */
/*===========================================================================*/

/** Fan PWM frequency in Hz */
#define BMS_FAN_PWM_FREQ_HZ         25000u

/** Minimum fan duty cycle (%) when active */
#define BMS_FAN_MIN_DUTY            30u

/** Maximum fan duty cycle (%) */
#define BMS_FAN_MAX_DUTY            100u

/** PWM timer channel for fan control */
#define PWM_TIMER_CHANNEL           0u

/*===========================================================================*/
/* Communication Configuration                                               */
/*===========================================================================*/

/** Debug UART baud rate */
#define BMS_DEBUG_UART_BAUD         115200u

/** SPI clock speed for MC33664B (Hz) */
#define BMS_SPI_CLOCK_HZ            1000000u

/** I2C clock speed for LCD (Hz) */
#define BMS_I2C_CLOCK_HZ            100000u

/** LCD I2C address (7-bit) */
#define BMS_LCD_I2C_ADDR            0x27u

/*===========================================================================*/
/* Feature Enable/Disable                                                    */
/*===========================================================================*/

/** Enable LCD display */
#define BMS_ENABLE_LCD              1

/** Enable cell balancing */
#define BMS_ENABLE_BALANCING        1

/** Enable functional safety supervisor */
#define BMS_ENABLE_FUSA             1

/** Enable watchdog */
#define BMS_ENABLE_WATCHDOG         1

/** Enable debug UART output */
#define BMS_ENABLE_DEBUG_UART       1

/*===========================================================================*/
/* Functional Safety (FuSa) Configuration - ISO 26262                        */
/*===========================================================================*/

/** ASIL level for this system (informational) */
#define BMS_FUSA_ASIL_LEVEL         "ASIL-C"

/** Fault tolerance time interval (FTTI) in ms */
#define BMS_FUSA_FTTI_MS            100u

/** Maximum consecutive communication failures before fault */
#define BMS_FUSA_MAX_COMM_FAILURES  3u

/** Plausibility check: minimum valid voltage reading (mV) */
#define BMS_FUSA_VOLT_PLAUSIBLE_MIN 1500u

/** Plausibility check: maximum valid voltage reading (mV) */
#define BMS_FUSA_VOLT_PLAUSIBLE_MAX 5500u

/** Plausibility check: minimum valid temperature (°C) */
#define BMS_FUSA_TEMP_PLAUSIBLE_MIN (-50)

/** Plausibility check: maximum valid temperature (°C) */
#define BMS_FUSA_TEMP_PLAUSIBLE_MAX 150

/** Maximum allowed voltage gradient (mV/100ms) */
#define BMS_FUSA_VOLT_GRADIENT_MAX  100u

/** Maximum allowed temperature gradient (°C/s) */
#define BMS_FUSA_TEMP_GRADIENT_MAX  5

/** Number of redundant checks required for critical decisions */
#define BMS_FUSA_REDUNDANCY_COUNT   2u

/** Alive counter timeout (supervision cycles) */
#define BMS_FUSA_ALIVE_TIMEOUT      5u

/** CRC polynomial for data integrity (CRC-8) */
#define BMS_FUSA_CRC_POLY           0x07u

/*===========================================================================*/
/* Diagnostic Thresholds                                                     */
/*===========================================================================*/

/** Debounce count for overvoltage fault confirmation */
#define BMS_DIAG_OV_DEBOUNCE        3u

/** Debounce count for undervoltage fault confirmation */
#define BMS_DIAG_UV_DEBOUNCE        3u

/** Debounce count for overtemperature fault confirmation */
#define BMS_DIAG_OT_DEBOUNCE        3u

/** Debounce count for communication fault confirmation */
#define BMS_DIAG_COMM_DEBOUNCE      5u

#endif /* BMS_CONFIG_H */
