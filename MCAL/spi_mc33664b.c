/**
 * @file spi_mc33664b.c
 * @brief SPI Driver Implementation for KL25Z + MC33664B
 * @details Register-level implementation following KL25Z Reference Manual
 * 
 * Important Notes:
 * - KL25Z has 48MHz system clock, 24MHz bus clock
 * - SPI0 runs on bus clock
 * - MC33664B max SPI speed: 4MHz
 * - We configure for 1MHz (safe, reliable)
 * 
 * SPI Baud Rate Calculation:
 * BaudRate = BusClock / ((SPPR + 1) × 2^(SPR + 1))
 * Where: SPPR = Prescaler divisor (0-7)
 *        SPR = Rate divisor (0-8)
 */

#include "spi_mc33664b.h"

/* Timeout for SPI operations (iterations) */
#define SPI_TIMEOUT_COUNT   10000u

/* Bus clock frequency (KL25Z default configuration) */
#define BUS_CLOCK_HZ        24000000u

/**
 * @brief Calculate SPI baud rate divisor
 * @param target_baud Desired baud rate in Hz
 * @return Baud rate register value (BR)
 * 
 * Formula: BaudRate = BusClock / ((SPPR + 1) × 2^(SPR + 1))
 * We need to find SPPR and SPR that give closest to target_baud
 * 
 * For 1MHz from 24MHz bus clock:
 * 24000000 / 1000000 = 24
 * Try: SPPR=2, SPR=2 → (2+1) × 2^(2+1) = 3 × 8 = 24 ✓
 * BR = (SPPR << 4) | SPR = (2 << 4) | 2 = 0x22
 */
static uint8_t calculate_baud_rate(uint32_t target_baud) {
    /* Simplified: for 1MHz, return pre-calculated value */
    /* For production: implement full calculation */
    if (target_baud >= 1000000) {
        return 0x22;  /* ~1MHz */
    } else if (target_baud >= 500000) {
        return 0x32;  /* ~500kHz */
    } else {
        return 0x44;  /* ~250kHz */
    }
}

SPI_Status_t SPI_MC33664B_Init(const SPI_Config_t *config) {
    if (config == NULL) {
        return SPI_ERROR;
    }
    
    SPI_MemMapPtr spi = SPI0_BASE_PTR;
    
    /* Step 1: Enable clock gates */
    SIM_SCGC4 |= SIM_SCGC4_SPI0_MASK;    /* Enable SPI0 clock */
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;   /* Enable PORTC clock */
    
    /* Step 2: Configure SPI pins */
    /* PTC5 = SPI0_SCK  (ALT2) */
    /* PTC6 = SPI0_MOSI (ALT2) */
    /* PTC7 = SPI0_MISO (ALT2) */
    PORTC_PCR5 = PORT_PCR_MUX_ALT2;
    PORTC_PCR6 = PORT_PCR_MUX_ALT2;
    PORTC_PCR7 = PORT_PCR_MUX_ALT2;
    
    /* PTC4 = CS (GPIO, manual control for better timing) */
    PORTC_PCR4 = PORT_PCR_MUX_GPIO;
    GPIOC_PDDR |= (1u << GPIO_CS_PIN);   /* Set as output */
    SPI_MC33664B_CS_Deassert();          /* CS high (inactive) */
    
    /* Step 3: Disable SPI before configuration */
    spi->C1 = 0x00;
    
    /* Step 4: Configure baud rate */
    spi->BR = calculate_baud_rate(config->baudrate_hz);
    
    /* Step 5: Configure SPI control register 1 */
    uint8_t c1_value = 0;
    c1_value |= SPI_C1_SPE_MASK;   /* Enable SPI */
    c1_value |= SPI_C1_MSTR_MASK;  /* Master mode */
    
    /* Configure clock polarity and phase based on mode */
    switch (config->mode) {
        case 0:  /* CPOL=0, CPHA=0 */
            /* Both bits cleared by default */
            break;
        case 1:  /* CPOL=0, CPHA=1 */
            c1_value |= SPI_C1_CPHA_MASK;
            break;
        case 2:  /* CPOL=1, CPHA=0 */
            c1_value |= SPI_C1_CPOL_MASK;
            break;
        case 3:  /* CPOL=1, CPHA=1 */
            c1_value |= SPI_C1_CPOL_MASK | SPI_C1_CPHA_MASK;
            break;
        default:
            return SPI_ERROR;
    }
    
    /* Configure bit order */
    if (!config->msb_first) {
        c1_value |= SPI_C1_LSBFE_MASK;
    }
    
    spi->C1 = c1_value;
    
    /* Step 6: Configure control register 2 */
    spi->C2 = 0x00;  /* 8-bit mode, no interrupts */
    
    /* Step 7: Clear status flags by reading S and D registers */
    (void)spi->S;
    (void)spi->D;
    
    return SPI_OK;
}

SPI_Status_t SPI_MC33664B_TransferData(const uint8_t *tx_data, 
                                        uint8_t *rx_data, 
                                        uint16_t length) {
    if (tx_data == NULL || rx_data == NULL || length == 0) {
        return SPI_ERROR;
    }
    
    SPI_MemMapPtr spi = SPI0_BASE_PTR;
    uint32_t timeout;
    
    for (uint16_t i = 0; i < length; i++) {
        /* Wait for transmit buffer empty */
        timeout = SPI_TIMEOUT_COUNT;
        while (!(spi->S & SPI_S_SPTEF_MASK)) {
            if (--timeout == 0) {
                return SPI_TIMEOUT;
            }
        }
        
        /* Write data to transmit buffer */
        spi->D = tx_data[i];
        
        /* Wait for receive buffer full (transfer complete) */
        timeout = SPI_TIMEOUT_COUNT;
        while (!(spi->S & SPI_S_SPRF_MASK)) {
            if (--timeout == 0) {
                return SPI_TIMEOUT;
            }
        }
        
        /* Read received data */
        rx_data[i] = spi->D;
    }
    
    return SPI_OK;
}

void SPI_MC33664B_CS_Assert(void) {
    /* Pull CS low (active) */
    GPIOC_PDOR &= ~(1u << GPIO_CS_PIN);
}

void SPI_MC33664B_CS_Deassert(void) {
    /* Pull CS high (inactive) */
    GPIOC_PDOR |= (1u << GPIO_CS_PIN);
}

uint8_t SPI_MC33664B_ReadByte(void) {
    SPI_MemMapPtr spi = SPI0_BASE_PTR;
    uint32_t timeout;
    
    /* Wait for transmit buffer empty */
    timeout = SPI_TIMEOUT_COUNT;
    while (!(spi->S & SPI_S_SPTEF_MASK)) {
        if (--timeout == 0) return 0xFF;
    }
    
    /* Send dummy byte to generate clock */
    spi->D = 0xFF;
    
    /* Wait for receive buffer full */
    timeout = SPI_TIMEOUT_COUNT;
    while (!(spi->S & SPI_S_SPRF_MASK)) {
        if (--timeout == 0) return 0xFF;
    }
    
    /* Return received byte */
    return spi->D;
}

void SPI_MC33664B_WriteByte(uint8_t data) {
    SPI_MemMapPtr spi = SPI0_BASE_PTR;
    uint32_t timeout;
    
    /* Wait for transmit buffer empty */
    timeout = SPI_TIMEOUT_COUNT;
    while (!(spi->S & SPI_S_SPTEF_MASK)) {
        if (--timeout == 0) return;
    }
    
    /* Write data */
    spi->D = data;
    
    /* Wait for transfer complete */
    timeout = SPI_TIMEOUT_COUNT;
    while (!(spi->S & SPI_S_SPRF_MASK)) {
        if (--timeout == 0) return;
    }
    
    /* Read and discard received byte (clear flag) */
    (void)spi->D;
}