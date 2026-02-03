/**
 * @file fusa_supervisor.h
 * @brief Functional Safety Supervisor - Application Layer
 * @details ISO 26262 compliant safety supervision for EV-BMS
 *
 * Safety Mechanisms Implemented:
 * - Plausibility checks (voltage, temperature ranges)
 * - Gradient monitoring (rate of change limits)
 * - Redundant data verification
 * - Alive supervision (watchdog pattern)
 * - Safe state management
 * - Fault reaction timing (FTTI compliance)
 *
 * ASIL Classification: ASIL-C (per ISO 26262)
 * - Single point faults must be detected within FTTI
 * - Latent faults detected by diagnostic coverage
 */

#ifndef APP_FUSA_SUPERVISOR_H
#define APP_FUSA_SUPERVISOR_H

#include <stdint.h>
#include <stdbool.h>

/*===========================================================================*/
/* Safety State Definitions                                                  */
/*===========================================================================*/

/**
 * @brief System safety states (state machine)
 */
typedef enum {
    FUSA_STATE_INIT = 0,        /* Initialization/self-test */
    FUSA_STATE_NORMAL,          /* Normal operation */
    FUSA_STATE_DEGRADED,        /* Degraded operation (minor fault) */
    FUSA_STATE_SAFE,            /* Safe state (major fault) */
    FUSA_STATE_EMERGENCY        /* Emergency shutdown */
} FuSa_State_t;

/**
 * @brief Safety fault categories
 */
typedef enum {
    FUSA_FAULT_NONE = 0,
    FUSA_FAULT_VOLTAGE_PLAUSIBILITY,    /* Voltage outside sensor range */
    FUSA_FAULT_TEMP_PLAUSIBILITY,       /* Temperature outside sensor range */
    FUSA_FAULT_VOLTAGE_GRADIENT,        /* Voltage changing too fast */
    FUSA_FAULT_TEMP_GRADIENT,           /* Temperature changing too fast */
    FUSA_FAULT_VOLTAGE_CRITICAL,        /* Critical OV/UV condition */
    FUSA_FAULT_TEMP_CRITICAL,           /* Critical overtemperature */
    FUSA_FAULT_COMM_LOSS,               /* Communication lost with slaves */
    FUSA_FAULT_DATA_STALE,              /* Data not refreshed in time */
    FUSA_FAULT_REDUNDANCY_MISMATCH,     /* Redundant checks disagree */
    FUSA_FAULT_ALIVE_TIMEOUT,           /* Alive supervision timeout */
    FUSA_FAULT_RAM_CORRUPTION,          /* Memory integrity check failed */
    FUSA_FAULT_CONTACTOR_FEEDBACK,      /* Contactor state mismatch */
    FUSA_FAULT_COUNT
} FuSa_FaultType_t;

/**
 * @brief Fault severity levels
 */
typedef enum {
    FUSA_SEVERITY_INFO = 0,     /* Informational only */
    FUSA_SEVERITY_WARNING,      /* Warning, continue operation */
    FUSA_SEVERITY_DEGRADED,     /* Enter degraded mode */
    FUSA_SEVERITY_CRITICAL      /* Enter safe state immediately */
} FuSa_Severity_t;

/**
 * @brief Fault log entry structure
 */
typedef struct {
    FuSa_FaultType_t fault_type;
    FuSa_Severity_t severity;
    uint32_t timestamp_ms;
    uint16_t fault_data;        /* Additional fault-specific data */
    uint8_t cell_index;         /* Affected cell (if applicable) */
    bool active;                /* Fault currently active */
} FuSa_FaultEntry_t;

/**
 * @brief Safety status summary
 */
typedef struct {
    FuSa_State_t current_state;
    uint8_t active_fault_count;
    uint32_t last_check_ms;
    uint32_t uptime_ms;
    bool contactor_allowed;     /* Safe to close contactors */
    bool charging_allowed;      /* Safe to charge */
    bool discharging_allowed;   /* Safe to discharge */
} FuSa_Status_t;

/*===========================================================================*/
/* Function Prototypes                                                       */
/*===========================================================================*/

/**
 * @brief Initialize the Functional Safety Supervisor
 *
 * Performs initial self-test and enters INIT state.
 * Verifies RAM integrity with pattern test.
 */
void FuSa_Init(void);

/**
 * @brief Perform all safety checks (main cyclic function)
 * @return true if all checks passed, false if any fault detected
 *
 * Must be called cyclically at BMS_MEASUREMENT_PERIOD_MS rate.
 * Implements FTTI monitoring.
 */
bool FuSa_PerformSafetyChecks(void);

/**
 * @brief Get current safety status
 * @return true if system is in safe operational state
 */
bool FuSa_GetSafetyStatus(void);

/**
 * @brief Get detailed safety status
 * @param status Pointer to status structure to fill
 */
void FuSa_GetDetailedStatus(FuSa_Status_t *status);

/**
 * @brief Get current safety state
 * @return Current FuSa state machine state
 */
FuSa_State_t FuSa_GetState(void);

/**
 * @brief Check if specific operation is allowed
 * @param operation 0=contactor, 1=charge, 2=discharge
 * @return true if operation is safe
 */
bool FuSa_IsOperationAllowed(uint8_t operation);

/**
 * @brief Request transition to safe state
 * @param fault_type Fault causing the transition
 *
 * Forces immediate transition to safe state.
 * Opens contactors and disables all outputs.
 */
void FuSa_RequestSafeState(FuSa_FaultType_t fault_type);

/**
 * @brief Attempt recovery from degraded/safe state
 * @return true if recovery successful
 *
 * Only allowed if all faults have been cleared.
 */
bool FuSa_AttemptRecovery(void);

/**
 * @brief Clear a specific fault (after root cause addressed)
 * @param fault_type Fault to clear
 * @return true if fault was cleared
 */
bool FuSa_ClearFault(FuSa_FaultType_t fault_type);

/**
 * @brief Get fault log entry
 * @param index Fault log index
 * @param entry Pointer to store fault entry
 * @return true if entry exists
 */
bool FuSa_GetFaultEntry(uint8_t index, FuSa_FaultEntry_t *entry);

/**
 * @brief Get count of active faults
 * @return Number of currently active faults
 */
uint8_t FuSa_GetActiveFaultCount(void);

/**
 * @brief Update alive counter (for supervision)
 *
 * Must be called by each supervised task to indicate healthy operation.
 */
void FuSa_AliveIndication(void);

/**
 * @brief Perform RAM integrity check
 * @return true if RAM is OK
 *
 * Uses checkerboard pattern test on reserved area.
 */
bool FuSa_CheckRAMIntegrity(void);

/**
 * @brief Calculate CRC for data integrity verification
 * @param data Pointer to data
 * @param length Data length in bytes
 * @return CRC-8 value
 */
uint8_t FuSa_CalculateCRC8(const uint8_t *data, uint8_t length);

/**
 * @brief Emergency shutdown handler
 *
 * Called when unrecoverable fault detected.
 * Opens all contactors and enters infinite safe loop.
 */
void FuSa_EmergencyShutdown(void);

#endif /* APP_FUSA_SUPERVISOR_H */
