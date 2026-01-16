/**
 * @file spi_mc33664b.h
 * @brief SPI Driver for KL25Z communicating with MC33664B
 * @details Implements proper KL25Z SPI0 hardware driver following
 *          KL25 Sub-Family Reference Manual (Chapter 43: SPI)
 *          Target: MC33664B System Basis Chip
 * 
 * Hardware Configuration:
 * - SPI0 Module on KL25Z
 * - PTC5: SPI0_SCK  (Clock)
 * - PTC6: SPI0_MOSI (Master Out Slave In)
 * - PTC7: SPI0_MISO (Master In Slave Out)
 * - PTC4: SPI0_CS   (Chip Select - GPIO controlled)
 * 
 * SPI Configuration:
 * - Mode: 0 (CPOL=0, CPHA=0)
 * - Speed: 1 MHz (MC33664B supports up to 4MHz)
 * - MSB First
 * - 8-bit frames
 */

#ifndef SPI_MC33664B_H
#define SPI_MC33664B_H

#include <stdint.h>
#include <stdbool.h>

/* KL25Z SPI Hardware Registers (from KL25 Reference Manual) */
#define SPI0_BASE_PTR       ((SPI_MemMapPtr)0x40076000u)

/* SPI Register Offsets */
typedef struct {
    volatile uint8_t C1;      /* Control Register 1 */
    volatile uint8_t C2;      /* Control Register 2 */
    volatile uint8_t BR;      /* Baud Rate Register */
    volatile uint8_t S;       /* Status Register */
    uint8_t RESERVED0;
    volatile uint8_t D;       /* Data Register */
    uint8_t RESERVED1;
    volatile uint8_t M;       /* Match Register */
} SPI_MemMap;

typedef SPI_MemMap *SPI_MemMapPtr;

/* SPI Control Register 1 (SPI0_C1) Bits */
#define SPI_C1_SPIE_MASK    0x80u   /* SPI Interrupt Enable */
#define SPI_C1_SPE_MASK     0x40u   /* SPI System Enable */
#define SPI_C1_SPTIE_MASK   0x20u   /* SPI Transmit Interrupt Enable */
#define SPI_C1_MSTR_MASK    0x10u   /* Master/Slave Mode Select */
#define SPI_C1_CPOL_MASK    0x08u   /* Clock Polarity */
#define SPI_C1_CPHA_MASK    0x04u   /* Clock Phase */
#define SPI_C1_SSOE_MASK    0x02u   /* Slave Select Output Enable */
#define SPI_C1_LSBFE_MASK   0x01u   /* LSB First Enable */

/* SPI Status Register (SPI0_S) Bits */
#define SPI_S_SPRF_MASK     0x80u   /* SPI Read Buffer Full Flag */
#define SPI_S_SPMF_MASK     0x40u   /* SPI Match Flag */
#define SPI_S_SPTEF_MASK    0x20u   /* SPI Transmit Buffer Empty Flag */
#define SPI_S_MODF_MASK     0x10u   /* Master Mode Fault Flag */

/* Clock Gate Control */
#define SIM_SCGC4           (*(volatile uint32_t *)0x40048034u)
#define SIM_SCGC4_SPI0_MASK 0x00400000u

#define SIM_SCGC5           (*(volatile uint32_t *)0x40048038u)
#define SIM_SCGC5_PORTC_MASK 0x00000800u

/* Port Control Registers for SPI Pins */
#define PORTC_BASE          0x4004B000u
#define PORTC_PCR4          (*(volatile uint32_t *)(PORTC_BASE + 0x10))  /* CS */
#define PORTC_PCR5          (*(volatile uint32_t *)(PORTC_BASE + 0x14))  /* SCK */
#define PORTC_PCR6          (*(volatile uint32_t *)(PORTC_BASE + 0x18))  /* MOSI */
#define PORTC_PCR7          (*(volatile uint32_t *)(PORTC_BASE + 0x1C))  /* MISO */

#define PORT_PCR_MUX(x)     (((uint32_t)(x) << 8) & 0x00000700u)
#define PORT_PCR_MUX_GPIO   PORT_PCR_MUX(1)
#define PORT_PCR_MUX_ALT2   PORT_PCR_MUX(2)  /* SPI function */

/* GPIO for CS control */
#define GPIOC_PDOR          (*(volatile uint32_t *)0x400FF080u)
#define GPIOC_PDDR          (*(volatile uint32_t *)0x400FF094u)
#define GPIO_CS_PIN         4u

/* Return status codes */
typedef enum {
    SPI_OK = 0,
    SPI_ERROR = 1,
    SPI_TIMEOUT = 2,
    SPI_BUSY = 3
} SPI_Status_t;

/* SPI Configuration Structure */
typedef struct {
    uint32_t baudrate_hz;       /* Desired baud rate in Hz */
    uint8_t  mode;              /* SPI mode (0-3) */
    bool     msb_first;         /* true = MSB first, false = LSB first */
} SPI_Config_t;

/* Function Prototypes */

/**
 * @brief Initialize SPI0 hardware for MC33664B communication
 * @param config Pointer to SPI configuration structure
 * @return SPI_OK on success, SPI_ERROR on failure
 * 
 * This function:
 * 1. Enables clock to SPI0 and PORTC
 * 2. Configures pins: PTC5(SCK), PTC6(MOSI), PTC7(MISO), PTC4(CS)
 * 3. Sets up SPI0 as master with specified baud rate and mode
 * 4. Initializes CS as GPIO output (high = inactive)
 */
SPI_Status_t SPI_MC33664B_Init(const SPI_Config_t *config);

/**
 * @brief Transmit and receive data simultaneously (full duplex)
 * @param tx_data Pointer to transmit buffer
 * @param rx_data Pointer to receive buffer
 * @param length Number of bytes to transfer
 * @return SPI_OK on success, error code otherwise
 * 
 * This is the main SPI communication function. It sends and receives
 * data byte-by-byte with proper flag checking and timeout protection.
 */
SPI_Status_t SPI_MC33664B_TransferData(const uint8_t *tx_data, 
                                        uint8_t *rx_data, 
                                        uint16_t length);

/**
 * @brief Assert (activate) chip select - Pull CS low
 * CS must be asserted before any SPI transaction
 */
void SPI_MC33664B_CS_Assert(void);

/**
 * @brief Deassert (deactivate) chip select - Pull CS high
 * CS must be deasserted after SPI transaction completes
 */
void SPI_MC33664B_CS_Deassert(void);

/**
 * @brief Read single byte via SPI (sends dummy 0xFF)
 * @return Received byte
 */
uint8_t SPI_MC33664B_ReadByte(void);

/**
 * @brief Write single byte via SPI
 * @param data Byte to transmit
 */
void SPI_MC33664B_WriteByte(uint8_t data);

#endif /* SPI_MC33664B_H */