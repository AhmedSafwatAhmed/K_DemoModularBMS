/**
 * @file i2c.h
 * @brief I2C Driver for KL25Z
 * @details Register-level I2C implementation for LCD communication
 *
 * KL25Z has two I2C modules:
 * - I2C0: PTE24 (SCL), PTE25 (SDA) - ALT5
 * - I2C1: PTE0 (SCL), PTE1 (SDA) - ALT6
 *
 * I2C0 Configuration for LCD:
 * - Standard mode: 100kHz
 * - 7-bit addressing
 * - Master mode only
 */

#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdbool.h>

/*===========================================================================*/
/* I2C Register Definitions                                                  */
/*===========================================================================*/

typedef struct {
    volatile uint8_t A1;        /* Address Register 1 */
    volatile uint8_t F;         /* Frequency Divider */
    volatile uint8_t C1;        /* Control Register 1 */
    volatile uint8_t S;         /* Status Register */
    volatile uint8_t D;         /* Data Register */
    volatile uint8_t C2;        /* Control Register 2 */
    volatile uint8_t FLT;       /* Filter Register */
    volatile uint8_t RA;        /* Range Address Register */
    volatile uint8_t SMB;       /* SMBus Control and Status */
    volatile uint8_t A2;        /* Address Register 2 */
    volatile uint8_t SLTH;      /* SCL Low Timeout High */
    volatile uint8_t SLTL;      /* SCL Low Timeout Low */
} I2C_Type;

#define I2C0                    ((I2C_Type *)0x40066000u)
#define I2C1                    ((I2C_Type *)0x40067000u)

/*===========================================================================*/
/* I2C Register Bit Definitions                                              */
/*===========================================================================*/

/* Control Register 1 (I2Cx_C1) */
#define I2C_C1_DMAEN_MASK       (1u << 0)   /* DMA Enable */
#define I2C_C1_WUEN_MASK        (1u << 1)   /* Wakeup Enable */
#define I2C_C1_RSTA_MASK        (1u << 2)   /* Repeat Start */
#define I2C_C1_TXAK_MASK        (1u << 3)   /* Transmit Acknowledge Enable */
#define I2C_C1_TX_MASK          (1u << 4)   /* Transmit Mode Select */
#define I2C_C1_MST_MASK         (1u << 5)   /* Master Mode Select */
#define I2C_C1_IICIE_MASK       (1u << 6)   /* I2C Interrupt Enable */
#define I2C_C1_IICEN_MASK       (1u << 7)   /* I2C Enable */

/* Status Register (I2Cx_S) */
#define I2C_S_RXAK_MASK         (1u << 0)   /* Receive Acknowledge */
#define I2C_S_IICIF_MASK        (1u << 1)   /* Interrupt Flag */
#define I2C_S_SRW_MASK          (1u << 2)   /* Slave Read/Write */
#define I2C_S_RAM_MASK          (1u << 3)   /* Range Address Match */
#define I2C_S_ARBL_MASK         (1u << 4)   /* Arbitration Lost */
#define I2C_S_BUSY_MASK         (1u << 5)   /* Bus Busy */
#define I2C_S_IAAS_MASK         (1u << 6)   /* Addressed as Slave */
#define I2C_S_TCF_MASK          (1u << 7)   /* Transfer Complete Flag */

/*===========================================================================*/
/* Clock Gating                                                              */
/*===========================================================================*/

#define SIM_SCGC4_I2C           (*(volatile uint32_t *)0x40048034u)
#define SIM_SCGC4_I2C0_MSK      (1u << 6)
#define SIM_SCGC4_I2C1_MSK      (1u << 7)

#define SIM_SCGC5_I2C           (*(volatile uint32_t *)0x40048038u)
#define SIM_SCGC5_PORTE_MSK     (1u << 13)

/* Port E pin configuration */
#define PORTE_PCR24             (*(volatile uint32_t *)(0x4004D000u + 24*4))
#define PORTE_PCR25             (*(volatile uint32_t *)(0x4004D000u + 25*4))
#define PORT_PCR_MUX_ALT5       (5u << 8)

/*===========================================================================*/
/* I2C Configuration Types                                                   */
/*===========================================================================*/

typedef enum {
    I2C_OK = 0,
    I2C_ERROR = 1,
    I2C_TIMEOUT = 2,
    I2C_NACK = 3,
    I2C_ARB_LOST = 4,
    I2C_BUSY = 5
} I2C_Status_t;

typedef struct {
    uint32_t clock_hz;          /* I2C clock frequency (typically 100000) */
} I2C_Config_t;

/*===========================================================================*/
/* Function Prototypes                                                       */
/*===========================================================================*/

/**
 * @brief Initialize I2C0 peripheral
 * @param config Pointer to I2C configuration
 * @return I2C_OK on success
 *
 * Configures I2C0 as master with specified clock frequency.
 * Pins PTE24 (SCL) and PTE25 (SDA) are configured automatically.
 */
I2C_Status_t I2C_Init(const I2C_Config_t *config);

/**
 * @brief Write data to I2C slave
 * @param slave_addr 7-bit slave address
 * @param data Pointer to data buffer
 * @param length Number of bytes to write
 * @return I2C_OK on success, error code otherwise
 */
I2C_Status_t I2C_Write(uint8_t slave_addr, const uint8_t *data, uint16_t length);

/**
 * @brief Read data from I2C slave
 * @param slave_addr 7-bit slave address
 * @param data Pointer to receive buffer
 * @param length Number of bytes to read
 * @return I2C_OK on success, error code otherwise
 */
I2C_Status_t I2C_Read(uint8_t slave_addr, uint8_t *data, uint16_t length);

/**
 * @brief Write then read (combined transaction)
 * @param slave_addr 7-bit slave address
 * @param tx_data Pointer to transmit data
 * @param tx_len Number of bytes to transmit
 * @param rx_data Pointer to receive buffer
 * @param rx_len Number of bytes to read
 * @return I2C_OK on success
 *
 * Performs write followed by repeated start and read.
 * Useful for register reads where address must be sent first.
 */
I2C_Status_t I2C_WriteRead(uint8_t slave_addr,
                           const uint8_t *tx_data, uint16_t tx_len,
                           uint8_t *rx_data, uint16_t rx_len);

/**
 * @brief Write single byte to I2C slave
 * @param slave_addr 7-bit slave address
 * @param data Byte to write
 * @return I2C_OK on success
 */
I2C_Status_t I2C_WriteByte(uint8_t slave_addr, uint8_t data);

/**
 * @brief Check if I2C bus is busy
 * @return true if busy, false otherwise
 */
bool I2C_IsBusy(void);

/**
 * @brief Reset I2C peripheral
 *
 * Resets I2C module in case of bus lockup.
 */
void I2C_Reset(void);

#endif /* I2C_H */
