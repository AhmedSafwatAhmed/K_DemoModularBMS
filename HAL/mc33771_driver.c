/**
 * @file mc33771_driver.c
 * @brief MC33771C Battery Cell Controller Driver Implementation
 * @details Communicates with MC33771C via MC33664B transceiver over SPI
 */

#include "mc33771_driver.h"
#include "spi_mc33664b.h"
#include "timer.h"
#include <string.h>

/*===========================================================================*/
/* Private Definitions                                                       */
/*===========================================================================*/

/* MC33664B Frame Format */
#define FRAME_SIZE              5u      /* 40-bit frames */
#define CRC_POLYNOMIAL          0x2Fu   /* CRC-8 polynomial for MC33771 */

/* ADC conversion factor: 16-bit ADC, 5V reference */
#define ADC_RESOLUTION          65536u
#define ADC_VREF_MV             5000u
#define ADC_TO_MV(x)            ((uint32_t)(x) * ADC_VREF_MV / ADC_RESOLUTION)

/* Temperature sensor conversion (NTC thermistor lookup simplified) */
#define TEMP_OFFSET             40      /* Offset for signed temperature */

/*===========================================================================*/
/* Private Variables                                                         */
/*===========================================================================*/

static SlaveData_t slave_data[BMS_NUM_SLAVES];
static bool driver_initialized = false;
static uint8_t comm_failure_count[BMS_NUM_SLAVES];

/*===========================================================================*/
/* Private Functions                                                         */
/*===========================================================================*/

/**
 * @brief Calculate CRC-8 for MC33771 communication
 */
uint8_t SlaveIF_CalculateCRC(const uint8_t *data, uint8_t length) {
    uint8_t crc = 0x00u;

    for (uint8_t i = 0u; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0u; bit < 8u; bit++) {
            if (crc & 0x80u) {
                crc = (crc << 1) ^ CRC_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

/**
 * @brief Build a read command frame
 */
static void build_read_frame(uint8_t slave_addr, uint8_t reg_addr, uint8_t *frame) {
    frame[0] = (slave_addr & 0x3Fu);                    /* Device address */
    frame[1] = MC33771_CMD_READ | (reg_addr & 0x7Fu);   /* Read command + register */
    frame[2] = 0x00u;                                   /* Data high (empty for read) */
    frame[3] = 0x00u;                                   /* Data low (empty for read) */
    frame[4] = SlaveIF_CalculateCRC(frame, 4u);         /* CRC */
}

/**
 * @brief Build a write command frame
 */
static void build_write_frame(uint8_t slave_addr, uint8_t reg_addr,
                              uint16_t data, uint8_t *frame) {
    frame[0] = (slave_addr & 0x3Fu);                    /* Device address */
    frame[1] = MC33771_CMD_WRITE | (reg_addr & 0x7Fu);  /* Write command + register */
    frame[2] = (uint8_t)(data >> 8);                    /* Data high */
    frame[3] = (uint8_t)(data & 0xFFu);                 /* Data low */
    frame[4] = SlaveIF_CalculateCRC(frame, 4u);         /* CRC */
}

/**
 * @brief Send frame and receive response
 */
static SlaveIF_Status_t transceive_frame(const uint8_t *tx_frame, uint8_t *rx_frame) {
    SPI_MC33664B_CS_Assert();
    TIMER_DelayUs(5);  /* CS setup time */

    SPI_Status_t spi_status = SPI_MC33664B_TransferData(tx_frame, rx_frame, FRAME_SIZE);

    TIMER_DelayUs(5);  /* CS hold time */
    SPI_MC33664B_CS_Deassert();

    if (spi_status != SPI_OK) {
        return SLAVE_COMM_ERROR;
    }

    /* Verify CRC of response */
    uint8_t calc_crc = SlaveIF_CalculateCRC(rx_frame, 4u);
    if (calc_crc != rx_frame[4]) {
        return SLAVE_CRC_ERROR;
    }

    return SLAVE_OK;
}

/**
 * @brief Read a register from a slave
 */
static SlaveIF_Status_t read_register(uint8_t slave_id, uint8_t reg_addr, uint16_t *value) {
    uint8_t tx_frame[FRAME_SIZE];
    uint8_t rx_frame[FRAME_SIZE];

    /* Build and send read command */
    build_read_frame(slave_id + 1u, reg_addr, tx_frame);  /* Slave addresses are 1-based */

    SlaveIF_Status_t status = transceive_frame(tx_frame, rx_frame);
    if (status != SLAVE_OK) {
        return status;
    }

    /* Small delay for slave to prepare response */
    TIMER_DelayUs(50);

    /* Read response frame */
    memset(tx_frame, 0xFFu, FRAME_SIZE);  /* Dummy data for clock generation */
    status = transceive_frame(tx_frame, rx_frame);
    if (status != SLAVE_OK) {
        return status;
    }

    /* Extract data from response */
    *value = ((uint16_t)rx_frame[2] << 8) | rx_frame[3];

    return SLAVE_OK;
}

/**
 * @brief Write a register to a slave
 */
static SlaveIF_Status_t write_register(uint8_t slave_id, uint8_t reg_addr, uint16_t value) {
    uint8_t tx_frame[FRAME_SIZE];
    uint8_t rx_frame[FRAME_SIZE];

    build_write_frame(slave_id + 1u, reg_addr, value, tx_frame);

    return transceive_frame(tx_frame, rx_frame);
}

/**
 * @brief Convert raw ADC value to millivolts
 */
static uint16_t adc_to_cell_voltage(uint16_t raw_adc) {
    /* MC33771 cell voltage measurement:
     * Full scale = 5V, 16-bit resolution
     * Voltage (mV) = raw * 5000 / 65536
     */
    return (uint16_t)ADC_TO_MV(raw_adc);
}

/**
 * @brief Convert raw ADC to temperature (simplified NTC conversion)
 */
static int8_t adc_to_temperature(uint16_t raw_adc) {
    /* Simplified linear approximation for 10K NTC
     * In production: use lookup table or Steinhart-Hart equation
     *
     * Approximate: T = 25 + (ADC_mid - raw) * scale
     * For demo: map ADC range to -20°C to +80°C
     */
    int32_t temp = 25 - ((int32_t)raw_adc - 32768) / 400;

    if (temp > 125) temp = 125;
    if (temp < -40) temp = -40;

    return (int8_t)temp;
}

/*===========================================================================*/
/* Public Function Implementations                                           */
/*===========================================================================*/

bool SlaveIF_Init(void) {
    /* Initialize SPI for MC33664B */
    SPI_Config_t spi_config = {
        .baudrate_hz = BMS_SPI_CLOCK_HZ,
        .mode = 0,          /* SPI Mode 0: CPOL=0, CPHA=0 */
        .msb_first = true
    };

    if (SPI_MC33664B_Init(&spi_config) != SPI_OK) {
        return false;
    }

    /* Initialize slave data structures */
    memset(slave_data, 0, sizeof(slave_data));
    memset(comm_failure_count, 0, sizeof(comm_failure_count));

    /* Small delay for MC33664B power-up */
    TIMER_DelayMs(10);

    /* Wake up the daisy chain */
    if (!SlaveIF_WakeUp()) {
        return false;
    }

    /* Verify communication with all slaves */
    uint8_t responding = SlaveIF_CheckCommunication();
    if (responding < BMS_NUM_SLAVES) {
        /* Not all slaves responding - continue with partial chain for demo */
    }

    /* Configure each slave */
    for (uint8_t i = 0u; i < BMS_NUM_SLAVES; i++) {
        /* Enable cell voltage measurements for configured cells */
        write_register(i, MC33771_OV_UV_EN_REG, 0x000Fu);  /* Enable cells 1-4 */

        /* Configure ADC for continuous measurement */
        write_register(i, MC33771_ADC_CFG_REG, 0x0003u);

        /* Configure GPIO for temperature sensing */
        write_register(i, MC33771_GPIO_CFG1_REG, 0x0003u);  /* AN0, AN1 as analog inputs */
    }

    driver_initialized = true;

    return true;
}

bool SlaveIF_RequestMeasurements(uint8_t slave_id) {
    if (!driver_initialized || slave_id >= BMS_NUM_SLAVES) {
        return false;
    }

    /* Trigger measurement on slave:
     * Enable cell voltage + analog input + pack voltage measurement
     */
    uint16_t meas_ctrl = MC33771_MEAS_START |
                         MC33771_MEAS_CELL_EN |
                         MC33771_MEAS_AN_EN |
                         MC33771_MEAS_VPWR_EN;

    SlaveIF_Status_t status = write_register(slave_id, MC33771_MEAS_CTRL_REG, meas_ctrl);

    return (status == SLAVE_OK);
}

bool SlaveIF_GetSlaveData(uint8_t slave_id, SlaveData_t *data) {
    if (!driver_initialized || slave_id >= BMS_NUM_SLAVES || data == NULL) {
        return false;
    }

    /* Read cell voltages */
    SlaveIF_Status_t status = SlaveIF_ReadCellVoltages(slave_id, slave_data[slave_id].cell_voltages_mV);
    if (status != SLAVE_OK) {
        comm_failure_count[slave_id]++;
        slave_data[slave_id].data_valid = false;
        return false;
    }

    /* Read temperatures */
    status = SlaveIF_ReadTemperatures(slave_id, slave_data[slave_id].temperatures_C);
    if (status != SLAVE_OK) {
        comm_failure_count[slave_id]++;
        slave_data[slave_id].data_valid = false;
        return false;
    }

    /* Update metadata */
    slave_data[slave_id].data_valid = true;
    slave_data[slave_id].last_update_ms = TIMER_GetTick();
    comm_failure_count[slave_id] = 0;

    /* Copy data to output */
    memcpy(data, &slave_data[slave_id], sizeof(SlaveData_t));

    return true;
}

SlaveIF_Status_t SlaveIF_ReadCellVoltages(uint8_t slave_id, uint16_t *voltages) {
    if (!driver_initialized || slave_id >= BMS_NUM_SLAVES || voltages == NULL) {
        return SLAVE_ERROR;
    }

    /* Read cell voltage registers */
    for (uint8_t cell = 0u; cell < BMS_CELLS_PER_SLAVE; cell++) {
        uint16_t raw_value;
        uint8_t reg_addr = MC33771_CELL1_REG + cell;

        SlaveIF_Status_t status = read_register(slave_id, reg_addr, &raw_value);
        if (status != SLAVE_OK) {
            return status;
        }

        voltages[cell] = adc_to_cell_voltage(raw_value);
    }

    return SLAVE_OK;
}

SlaveIF_Status_t SlaveIF_ReadTemperatures(uint8_t slave_id, int8_t *temps) {
    if (!driver_initialized || slave_id >= BMS_NUM_SLAVES || temps == NULL) {
        return SLAVE_ERROR;
    }

    /* Read analog input registers (AN0, AN1 for temperatures) */
    for (uint8_t sensor = 0u; sensor < BMS_TEMPS_PER_SLAVE; sensor++) {
        uint16_t raw_value;
        uint8_t reg_addr = MC33771_AN0_REG + sensor;

        SlaveIF_Status_t status = read_register(slave_id, reg_addr, &raw_value);
        if (status != SLAVE_OK) {
            return status;
        }

        temps[sensor] = adc_to_temperature(raw_value);
    }

    return SLAVE_OK;
}

SlaveIF_Status_t SlaveIF_EnableBalancing(uint8_t slave_id, const BalanceConfig_t *config) {
    if (!driver_initialized || slave_id >= BMS_NUM_SLAVES || config == NULL) {
        return SLAVE_ERROR;
    }

    /* Write balancing timer */
    SlaveIF_Status_t status = write_register(slave_id, MC33771_CB_TIMER_REG, config->balance_time_s);
    if (status != SLAVE_OK) {
        return status;
    }

    /* Write cell balancing mask (which cells to balance) */
    status = write_register(slave_id, MC33771_CB1_CFG_REG, config->balance_mask);

    return status;
}

SlaveIF_Status_t SlaveIF_DisableBalancing(uint8_t slave_id) {
    if (!driver_initialized || slave_id >= BMS_NUM_SLAVES) {
        return SLAVE_ERROR;
    }

    /* Clear balancing mask */
    return write_register(slave_id, MC33771_CB1_CFG_REG, 0x0000u);
}

SlaveIF_Status_t SlaveIF_ReadFaultStatus(uint8_t slave_id, uint8_t *fault_status) {
    if (!driver_initialized || slave_id >= BMS_NUM_SLAVES || fault_status == NULL) {
        return SLAVE_ERROR;
    }

    uint16_t status1, status2;

    SlaveIF_Status_t status = read_register(slave_id, MC33771_STATUS1_REG, &status1);
    if (status != SLAVE_OK) {
        return status;
    }

    status = read_register(slave_id, MC33771_STATUS2_REG, &status2);
    if (status != SLAVE_OK) {
        return status;
    }

    /* Combine fault flags into single byte */
    *fault_status = 0u;
    if (status1 & 0x0001u) *fault_status |= 0x01u;  /* Overvoltage */
    if (status1 & 0x0002u) *fault_status |= 0x02u;  /* Undervoltage */
    if (status1 & 0x0004u) *fault_status |= 0x04u;  /* Overtemperature */
    if (status2 & 0x0001u) *fault_status |= 0x08u;  /* Communication error */

    slave_data[slave_id].fault_status = *fault_status;

    return SLAVE_OK;
}

SlaveIF_Status_t SlaveIF_ClearFaults(uint8_t slave_id) {
    if (!driver_initialized || slave_id >= BMS_NUM_SLAVES) {
        return SLAVE_ERROR;
    }

    /* Write 1s to clear fault flags in status registers */
    SlaveIF_Status_t status = write_register(slave_id, MC33771_STATUS1_REG, 0xFFFFu);
    if (status != SLAVE_OK) {
        return status;
    }

    status = write_register(slave_id, MC33771_STATUS2_REG, 0xFFFFu);
    if (status != SLAVE_OK) {
        return status;
    }

    slave_data[slave_id].fault_status = 0u;

    return SLAVE_OK;
}

uint8_t SlaveIF_CheckCommunication(void) {
    uint8_t responding = 0u;

    for (uint8_t i = 0u; i < BMS_NUM_SLAVES; i++) {
        uint16_t init_reg;

        /* Try to read INIT register - should return non-zero if slave present */
        SlaveIF_Status_t status = read_register(i, MC33771_INIT_REG, &init_reg);

        if (status == SLAVE_OK && init_reg != 0u) {
            responding++;
        }
    }

    return responding;
}

bool SlaveIF_WakeUp(void) {
    /* Send wake-up pattern via SPI
     * MC33664B requires specific pulse pattern to wake up TPL
     */

    /* Assert CS for wake-up pulse */
    SPI_MC33664B_CS_Assert();
    TIMER_DelayMs(1);
    SPI_MC33664B_CS_Deassert();

    /* Wait for slaves to wake up */
    TIMER_DelayMs(10);

    /* Send dummy frame to synchronize */
    uint8_t dummy_tx[FRAME_SIZE] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu};
    uint8_t dummy_rx[FRAME_SIZE];

    SPI_MC33664B_CS_Assert();
    SPI_MC33664B_TransferData(dummy_tx, dummy_rx, FRAME_SIZE);
    SPI_MC33664B_CS_Deassert();

    TIMER_DelayMs(5);

    return true;
}

bool SlaveIF_Sleep(void) {
    /* Send sleep command to all slaves */
    for (uint8_t i = 0u; i < BMS_NUM_SLAVES; i++) {
        write_register(i, MC33771_WAKEUP_REG, 0x0000u);
    }

    driver_initialized = false;

    return true;
}
