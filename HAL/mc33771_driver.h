/**
 * @file mc33771_driver.h
 * @brief MC33771C Battery Cell Controller Driver
 * @details High-level driver for NXP MC33771C communicating via MC33664B
 *
 * MC33771C Features:
 * - 14-cell voltage measurement (16-bit resolution)
 * - 7 temperature inputs (GPIO/thermistor)
 * - Passive cell balancing
 * - Fault detection and diagnostics
 * - Daisy chain support via TPL (Transformer Physical Layer)
 *
 * Communication:
 * - Master -> MC33664B (SPI) -> MC33771C chain (TPL)
 * - 40-bit frame format: CMD + DATA + CRC
 */

#ifndef MC33771_DRIVER_H
#define MC33771_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "bms_config.h"

/*===========================================================================*/
/* MC33771 Register Addresses                                                */
/*===========================================================================*/

/* System Registers */
#define MC33771_INIT_REG            0x01u
#define MC33771_STATUS1_REG         0x02u
#define MC33771_STATUS2_REG         0x03u
#define MC33771_STATUS3_REG         0x04u
#define MC33771_FAULT1_MASK_REG     0x05u
#define MC33771_FAULT2_MASK_REG     0x06u
#define MC33771_FAULT3_MASK_REG     0x07u
#define MC33771_WAKEUP_REG          0x08u
#define MC33771_CELL_OV_FLT_REG     0x09u
#define MC33771_CELL_UV_FLT_REG     0x0Au

/* Configuration Registers */
#define MC33771_GPIO_CFG1_REG       0x0Bu
#define MC33771_GPIO_CFG2_REG       0x0Cu
#define MC33771_ADC_CFG_REG         0x0Du
#define MC33771_ADC2_OFFSET_REG     0x0Eu
#define MC33771_OV_UV_EN_REG        0x0Fu

/* Measurement Control */
#define MC33771_MEAS_CTRL_REG       0x10u
#define MC33771_MEAS_STATUS_REG     0x11u

/* Cell Voltage Registers (14 cells) */
#define MC33771_CELL1_REG           0x12u
#define MC33771_CELL2_REG           0x13u
#define MC33771_CELL3_REG           0x14u
#define MC33771_CELL4_REG           0x15u
#define MC33771_CELL5_REG           0x16u
#define MC33771_CELL6_REG           0x17u
#define MC33771_CELL7_REG           0x18u
#define MC33771_CELL8_REG           0x19u
#define MC33771_CELL9_REG           0x1Au
#define MC33771_CELL10_REG          0x1Bu
#define MC33771_CELL11_REG          0x1Cu
#define MC33771_CELL12_REG          0x1Du
#define MC33771_CELL13_REG          0x1Eu
#define MC33771_CELL14_REG          0x1Fu

/* GPIO/Temperature Registers */
#define MC33771_AN0_REG             0x20u
#define MC33771_AN1_REG             0x21u
#define MC33771_AN2_REG             0x22u
#define MC33771_AN3_REG             0x23u
#define MC33771_AN4_REG             0x24u
#define MC33771_AN5_REG             0x25u
#define MC33771_AN6_REG             0x26u

/* Pack Voltage Register */
#define MC33771_VPWR_REG            0x27u

/* Cell Balancing Registers */
#define MC33771_CB1_CFG_REG         0x28u
#define MC33771_CB2_CFG_REG         0x29u
#define MC33771_CB3_CFG_REG         0x2Au
#define MC33771_CB_TIMER_REG        0x2Bu

/*===========================================================================*/
/* MC33771 Command Definitions                                               */
/*===========================================================================*/

#define MC33771_CMD_READ            0x00u
#define MC33771_CMD_WRITE           0x80u
#define MC33771_CMD_RESPONSE        0x40u

/* Measurement Control Bits */
#define MC33771_MEAS_START          0x01u
#define MC33771_MEAS_CELL_EN        0x02u
#define MC33771_MEAS_AN_EN          0x04u
#define MC33771_MEAS_VPWR_EN        0x08u

/*===========================================================================*/
/* Data Structures                                                           */
/*===========================================================================*/

/**
 * @brief Slave device data structure
 */
typedef struct {
    uint16_t cell_voltages_mV[BMS_CELLS_PER_SLAVE];
    int8_t   temperatures_C[BMS_TEMPS_PER_SLAVE];
    uint16_t pack_voltage_mV;
    uint8_t  fault_status;
    bool     data_valid;
    uint32_t last_update_ms;
} SlaveData_t;

/**
 * @brief Cell balancing configuration
 */
typedef struct {
    uint16_t balance_mask;      /* Bit mask of cells to balance */
    uint16_t balance_time_s;    /* Balancing duration in seconds */
} BalanceConfig_t;

/**
 * @brief Driver status codes
 */
typedef enum {
    SLAVE_OK = 0,
    SLAVE_ERROR = 1,
    SLAVE_TIMEOUT = 2,
    SLAVE_CRC_ERROR = 3,
    SLAVE_COMM_ERROR = 4,
    SLAVE_NOT_RESPONDING = 5
} SlaveIF_Status_t;

/*===========================================================================*/
/* Function Prototypes                                                       */
/*===========================================================================*/

/**
 * @brief Initialize the slave interface (SPI + MC33664B + MC33771 chain)
 * @return true on success, false on failure
 */
bool SlaveIF_Init(void);

/**
 * @brief Request measurements from a specific slave
 * @param slave_id Slave index (0 to BMS_NUM_SLAVES-1)
 * @return true if request was successful
 */
bool SlaveIF_RequestMeasurements(uint8_t slave_id);

/**
 * @brief Get measurement data from a slave
 * @param slave_id Slave index
 * @param data Pointer to store measurement data
 * @return true if data is valid
 */
bool SlaveIF_GetSlaveData(uint8_t slave_id, SlaveData_t *data);

/**
 * @brief Read cell voltages from a slave
 * @param slave_id Slave index
 * @param voltages Array to store voltages (mV)
 * @return SLAVE_OK on success
 */
SlaveIF_Status_t SlaveIF_ReadCellVoltages(uint8_t slave_id, uint16_t *voltages);

/**
 * @brief Read temperatures from a slave
 * @param slave_id Slave index
 * @param temps Array to store temperatures (°C)
 * @return SLAVE_OK on success
 */
SlaveIF_Status_t SlaveIF_ReadTemperatures(uint8_t slave_id, int8_t *temps);

/**
 * @brief Enable cell balancing
 * @param slave_id Slave index
 * @param config Balancing configuration
 * @return SLAVE_OK on success
 */
SlaveIF_Status_t SlaveIF_EnableBalancing(uint8_t slave_id, const BalanceConfig_t *config);

/**
 * @brief Disable cell balancing
 * @param slave_id Slave index
 * @return SLAVE_OK on success
 */
SlaveIF_Status_t SlaveIF_DisableBalancing(uint8_t slave_id);

/**
 * @brief Read fault status from a slave
 * @param slave_id Slave index
 * @param fault_status Pointer to store fault flags
 * @return SLAVE_OK on success
 */
SlaveIF_Status_t SlaveIF_ReadFaultStatus(uint8_t slave_id, uint8_t *fault_status);

/**
 * @brief Clear faults on a slave
 * @param slave_id Slave index
 * @return SLAVE_OK on success
 */
SlaveIF_Status_t SlaveIF_ClearFaults(uint8_t slave_id);

/**
 * @brief Check communication with all slaves
 * @return Number of responding slaves
 */
uint8_t SlaveIF_CheckCommunication(void);

/**
 * @brief Wake up slaves from sleep mode
 * @return true on success
 */
bool SlaveIF_WakeUp(void);

/**
 * @brief Put slaves into sleep mode
 * @return true on success
 */
bool SlaveIF_Sleep(void);

/**
 * @brief Calculate CRC for MC33771 communication
 * @param data Pointer to data buffer
 * @param length Number of bytes
 * @return CRC value
 */
uint8_t SlaveIF_CalculateCRC(const uint8_t *data, uint8_t length);

#endif /* MC33771_DRIVER_H */
