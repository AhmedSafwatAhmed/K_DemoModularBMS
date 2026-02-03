/**
 * @file i2c.c
 * @brief I2C Driver Implementation for KL25Z
 * @details I2C0 master driver for LCD communication
 */

#include "i2c.h"

/* Bus clock frequency (24MHz) */
#define BUS_CLOCK_HZ            24000000u

/* I2C timeout in iterations */
#define I2C_TIMEOUT_COUNT       100000u

/*===========================================================================*/
/* Private Functions                                                         */
/*===========================================================================*/

/**
 * @brief Wait for I2C interrupt flag with timeout
 */
static I2C_Status_t i2c_wait_flag(void) {
    uint32_t timeout = I2C_TIMEOUT_COUNT;

    while (!(I2C0->S & I2C_S_IICIF_MASK)) {
        if (--timeout == 0u) {
            return I2C_TIMEOUT;
        }
    }

    /* Clear interrupt flag */
    I2C0->S |= I2C_S_IICIF_MASK;

    return I2C_OK;
}

/**
 * @brief Generate start condition
 */
static void i2c_start(void) {
    I2C0->C1 |= I2C_C1_TX_MASK;     /* Transmit mode */
    I2C0->C1 |= I2C_C1_MST_MASK;    /* Master mode (generates START) */
}

/**
 * @brief Generate repeated start condition
 */
static void i2c_repeated_start(void) {
    I2C0->C1 |= I2C_C1_RSTA_MASK;
}

/**
 * @brief Generate stop condition
 */
static void i2c_stop(void) {
    I2C0->C1 &= ~I2C_C1_MST_MASK;   /* Slave mode (generates STOP) */
    I2C0->C1 &= ~I2C_C1_TX_MASK;    /* Receive mode */
}

/**
 * @brief Calculate I2C clock divider
 * @param target_freq Desired I2C clock frequency
 * @return ICR value for F register
 */
static uint8_t calculate_icr(uint32_t target_freq) {
    /*
     * I2C baud rate = bus_clock / (mul * SCL_divider)
     * For 100kHz at 24MHz bus clock:
     * 24000000 / (1 * 240) = 100000
     *
     * ICR lookup table (simplified):
     * ICR = 0x1F gives divider of 240 (mul=1)
     */
    if (target_freq >= 400000u) {
        return 0x12;  /* ~400kHz */
    } else if (target_freq >= 100000u) {
        return 0x1F;  /* ~100kHz */
    } else {
        return 0x27;  /* ~50kHz */
    }
}

/*===========================================================================*/
/* Public Function Implementations                                           */
/*===========================================================================*/

I2C_Status_t I2C_Init(const I2C_Config_t *config) {
    if (config == NULL) {
        return I2C_ERROR;
    }

    /* Enable clock to I2C0 and Port E */
    SIM_SCGC4_I2C |= SIM_SCGC4_I2C0_MSK;
    SIM_SCGC5_I2C |= SIM_SCGC5_PORTE_MSK;

    /* Configure PTE24 as I2C0_SCL (ALT5) */
    PORTE_PCR24 = PORT_PCR_MUX_ALT5;

    /* Configure PTE25 as I2C0_SDA (ALT5) */
    PORTE_PCR25 = PORT_PCR_MUX_ALT5;

    /* Disable I2C during configuration */
    I2C0->C1 = 0u;

    /* Set clock divider for desired frequency */
    I2C0->F = calculate_icr(config->clock_hz);

    /* Clear control register 2 */
    I2C0->C2 = 0u;

    /* Enable I2C */
    I2C0->C1 = I2C_C1_IICEN_MASK;

    return I2C_OK;
}

I2C_Status_t I2C_Write(uint8_t slave_addr, const uint8_t *data, uint16_t length) {
    if (data == NULL || length == 0u) {
        return I2C_ERROR;
    }

    I2C_Status_t status;

    /* Wait if bus is busy */
    uint32_t timeout = I2C_TIMEOUT_COUNT;
    while (I2C0->S & I2C_S_BUSY_MASK) {
        if (--timeout == 0u) {
            return I2C_BUSY;
        }
    }

    /* Generate START */
    i2c_start();

    /* Send slave address with write bit (0) */
    I2C0->D = (uint8_t)(slave_addr << 1);

    /* Wait for address transmission */
    status = i2c_wait_flag();
    if (status != I2C_OK) {
        i2c_stop();
        return status;
    }

    /* Check for ACK */
    if (I2C0->S & I2C_S_RXAK_MASK) {
        i2c_stop();
        return I2C_NACK;
    }

    /* Send data bytes */
    for (uint16_t i = 0u; i < length; i++) {
        I2C0->D = data[i];

        status = i2c_wait_flag();
        if (status != I2C_OK) {
            i2c_stop();
            return status;
        }

        if (I2C0->S & I2C_S_RXAK_MASK) {
            i2c_stop();
            return I2C_NACK;
        }
    }

    /* Generate STOP */
    i2c_stop();

    return I2C_OK;
}

I2C_Status_t I2C_Read(uint8_t slave_addr, uint8_t *data, uint16_t length) {
    if (data == NULL || length == 0u) {
        return I2C_ERROR;
    }

    I2C_Status_t status;

    /* Wait if bus is busy */
    uint32_t timeout = I2C_TIMEOUT_COUNT;
    while (I2C0->S & I2C_S_BUSY_MASK) {
        if (--timeout == 0u) {
            return I2C_BUSY;
        }
    }

    /* Generate START */
    i2c_start();

    /* Send slave address with read bit (1) */
    I2C0->D = (uint8_t)((slave_addr << 1) | 0x01);

    /* Wait for address transmission */
    status = i2c_wait_flag();
    if (status != I2C_OK) {
        i2c_stop();
        return status;
    }

    /* Check for ACK */
    if (I2C0->S & I2C_S_RXAK_MASK) {
        i2c_stop();
        return I2C_NACK;
    }

    /* Switch to receive mode */
    I2C0->C1 &= ~I2C_C1_TX_MASK;

    /* Configure ACK/NACK for multi-byte read */
    if (length == 1u) {
        I2C0->C1 |= I2C_C1_TXAK_MASK;  /* NACK for single byte */
    } else {
        I2C0->C1 &= ~I2C_C1_TXAK_MASK; /* ACK for multi-byte */
    }

    /* Dummy read to start reception */
    (void)I2C0->D;

    /* Read data bytes */
    for (uint16_t i = 0u; i < length; i++) {
        status = i2c_wait_flag();
        if (status != I2C_OK) {
            i2c_stop();
            return status;
        }

        /* Set NACK before reading second-to-last byte */
        if (i == (length - 2u)) {
            I2C0->C1 |= I2C_C1_TXAK_MASK;
        }

        /* Generate STOP before reading last byte */
        if (i == (length - 1u)) {
            i2c_stop();
        }

        data[i] = I2C0->D;
    }

    return I2C_OK;
}

I2C_Status_t I2C_WriteRead(uint8_t slave_addr,
                           const uint8_t *tx_data, uint16_t tx_len,
                           uint8_t *rx_data, uint16_t rx_len) {
    if (tx_data == NULL || rx_data == NULL || tx_len == 0u || rx_len == 0u) {
        return I2C_ERROR;
    }

    I2C_Status_t status;

    /* Wait if bus is busy */
    uint32_t timeout = I2C_TIMEOUT_COUNT;
    while (I2C0->S & I2C_S_BUSY_MASK) {
        if (--timeout == 0u) {
            return I2C_BUSY;
        }
    }

    /* Generate START */
    i2c_start();

    /* Send slave address with write bit */
    I2C0->D = (uint8_t)(slave_addr << 1);

    status = i2c_wait_flag();
    if (status != I2C_OK) {
        i2c_stop();
        return status;
    }

    if (I2C0->S & I2C_S_RXAK_MASK) {
        i2c_stop();
        return I2C_NACK;
    }

    /* Send write data */
    for (uint16_t i = 0u; i < tx_len; i++) {
        I2C0->D = tx_data[i];

        status = i2c_wait_flag();
        if (status != I2C_OK) {
            i2c_stop();
            return status;
        }

        if (I2C0->S & I2C_S_RXAK_MASK) {
            i2c_stop();
            return I2C_NACK;
        }
    }

    /* Generate repeated START */
    i2c_repeated_start();

    /* Send slave address with read bit */
    I2C0->D = (uint8_t)((slave_addr << 1) | 0x01);

    status = i2c_wait_flag();
    if (status != I2C_OK) {
        i2c_stop();
        return status;
    }

    if (I2C0->S & I2C_S_RXAK_MASK) {
        i2c_stop();
        return I2C_NACK;
    }

    /* Switch to receive mode */
    I2C0->C1 &= ~I2C_C1_TX_MASK;

    if (rx_len == 1u) {
        I2C0->C1 |= I2C_C1_TXAK_MASK;
    } else {
        I2C0->C1 &= ~I2C_C1_TXAK_MASK;
    }

    /* Dummy read */
    (void)I2C0->D;

    /* Read data */
    for (uint16_t i = 0u; i < rx_len; i++) {
        status = i2c_wait_flag();
        if (status != I2C_OK) {
            i2c_stop();
            return status;
        }

        if (i == (rx_len - 2u)) {
            I2C0->C1 |= I2C_C1_TXAK_MASK;
        }

        if (i == (rx_len - 1u)) {
            i2c_stop();
        }

        rx_data[i] = I2C0->D;
    }

    return I2C_OK;
}

I2C_Status_t I2C_WriteByte(uint8_t slave_addr, uint8_t data) {
    return I2C_Write(slave_addr, &data, 1u);
}

bool I2C_IsBusy(void) {
    return (I2C0->S & I2C_S_BUSY_MASK) != 0u;
}

void I2C_Reset(void) {
    /* Disable I2C */
    I2C0->C1 = 0u;

    /* Small delay */
    for (volatile uint32_t i = 0u; i < 1000u; i++) {
        __asm volatile ("nop");
    }

    /* Re-enable I2C */
    I2C0->C1 = I2C_C1_IICEN_MASK;
}
