/**
 * @file uart_kl25z.c
 * @brief UART Driver Implementation for KL25Z
 * @details Register-level implementation following KL25 Reference Manual
 * 
 * Baud Rate Calculation:
 * BaudRate = UARTClock / (16 × (SBR[12:0] + BRFD))
 * Where:
 *   UARTClock = System Clock (48MHz default)
 *   SBR = Baud Rate Modulo Divisor (13-bit)
 *   BRFD = Baud Rate Fine Adjust (5-bit, fractional)
 * 
 * For 115200 baud @ 48MHz:
 * SBR = 48000000 / (16 × 115200) = 26.04
 * SBR integer part = 26
 * BRFD = 0.04 × 32 = 1.28 ≈ 1
 */

#include "uart_kl25z.h"
#include "timer.h"

/* Timeout counter for operations */
#define UART_TIMEOUT_COUNT  50000u

/**
 * @brief Calculate SBR and BRFD values for desired baud rate
 * @param baudrate Desired baud rate
 * @param sbr Pointer to store SBR value
 * @param brfd Pointer to store BRFD value
 */
static void calculate_baud_divisors(uint32_t baudrate, uint16_t *sbr, uint8_t *brfd) {
    /* Calculate divisor: Divisor = Clock / (16 × BaudRate) */
    uint32_t divisor = UART_DEFAULT_CLOCK / (UART_CLOCK_DIV * baudrate);
    
    /* SBR is integer part (13 bits) */
    *sbr = (uint16_t)(divisor & 0x1FFF);
    
    /* BRFD is fractional part × 32 (5 bits) */
    /* For simplicity, we calculate the fractional part */
    uint32_t fractional = UART_DEFAULT_CLOCK - (*sbr * UART_CLOCK_DIV * baudrate);
    *brfd = (uint8_t)((fractional * 32) / (UART_CLOCK_DIV * baudrate));
    *brfd &= 0x1F;  /* Mask to 5 bits */
}

UART_Status_t UART_Init(const UART_Config_t *config) {
    if (config == NULL) {
        return UART_ERROR;
    }
    
    UART_MemMapPtr uart = UART0_BASE_PTR;
    
    /* Step 1: Enable clock gates */
    SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;   /* Enable UART0 clock */
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;   /* Enable PORTA clock */
    
    /* Step 2: Configure UART pins */
    /* PTA1 = UART0_RX (ALT2) */
    /* PTA2 = UART0_TX (ALT2) */
    PORTA_PCR1 = PORT_PCR_MUX(2);  /* RX */
    PORTA_PCR2 = PORT_PCR_MUX(2);  /* TX */
    
    /* Step 3: Disable UART during configuration */
    uart->C2 = 0x00;
    
    /* Step 4: Calculate and set baud rate */
    uint16_t sbr;
    uint8_t brfd;
    calculate_baud_divisors(config->baudrate, &sbr, &brfd);
    
    /* Set baud rate registers */
    uart->BDH = (uint8_t)((sbr >> 8) & 0x1F);  /* Upper 5 bits of SBR */
    uart->BDL = (uint8_t)(sbr & 0xFF);         /* Lower 8 bits of SBR */
    uart->C4 = brfd & 0x1F;                    /* Fine adjust (BRFA field) */
    
    /* Step 5: Configure data format */
    uart->C1 = 0x00;  /* Start with default: 8-bit, no parity */
    
    /* Configure data bits */
    if (config->data_bits == 9) {
        uart->C1 |= 0x10;  /* M bit: 9-bit mode */
    }
    
    /* Configure parity */
    if (config->parity != UART_PARITY_NONE) {
        uart->C1 |= 0x02;  /* PE bit: Parity enable */
        if (config->parity == UART_PARITY_ODD) {
            uart->C1 |= 0x01;  /* PT bit: Odd parity */
        }
    }
    
    /* Step 6: Clear error flags */
    /* Reading S1 then reading D clears error flags */
    (void)uart->S1;
    (void)uart->D;
    
    /* Step 7: Enable transmitter and receiver */
    uart->C2 = UART_C2_TE_MASK | UART_C2_RE_MASK;
    
    /* Small delay for UART to stabilize */
    for (volatile uint32_t i = 0; i < 1000; i++);
    
    return UART_OK;
}

UART_Status_t UART_TransmitByte(uint8_t data) {
    UART_MemMapPtr uart = UART0_BASE_PTR;
    uint32_t timeout = UART_TIMEOUT_COUNT;
    
    /* Wait for transmit buffer empty */
    while (!(uart->S1 & UART_S1_TDRE_MASK)) {
        if (--timeout == 0) {
            return UART_TIMEOUT;
        }
    }
    
    /* Write data to transmit register */
    uart->D = data;
    
    return UART_OK;
}

UART_Status_t UART_TransmitString(const char *str) {
    if (str == NULL) {
        return UART_ERROR;
    }
    
    while (*str != '\0') {
        UART_Status_t status = UART_TransmitByte(*str);
        if (status != UART_OK) {
            return status;
        }
        str++;
    }
    
    return UART_OK;
}

UART_Status_t UART_TransmitBuffer(const uint8_t *data, uint16_t length) {
    if (data == NULL || length == 0) {
        return UART_ERROR;
    }
    
    for (uint16_t i = 0; i < length; i++) {
        UART_Status_t status = UART_TransmitByte(data[i]);
        if (status != UART_OK) {
            return status;
        }
    }
    
    return UART_OK;
}

UART_Status_t UART_ReceiveByte(uint8_t *data, uint32_t timeout_ms) {
    if (data == NULL) {
        return UART_ERROR;
    }
    
    UART_MemMapPtr uart = UART0_BASE_PTR;
    
    if (timeout_ms == 0) {
        /* No timeout - wait indefinitely */
        while (!(uart->S1 & UART_S1_RDRF_MASK));
    } else {
        /* Wait with timeout */
        uint32_t start_tick = TIMER_GetTick();
        while (!(uart->S1 & UART_S1_RDRF_MASK)) {
            if ((TIMER_GetTick() - start_tick) >= timeout_ms) {
                return UART_TIMEOUT;
            }
        }
    }
    
    /* Check for errors */
    uint8_t status = uart->S1;
    if (status & (UART_S1_OR_MASK | UART_S1_NF_MASK | UART_S1_FE_MASK | UART_S1_PF_MASK)) {
        /* Clear error by reading D register */
        (void)uart->D;
        return UART_ERROR;
    }
    
    /* Read data from receive register */
    *data = uart->D;
    
    return UART_OK;
}

bool UART_DataAvailable(void) {
    UART_MemMapPtr uart = UART0_BASE_PTR;
    return (uart->S1 & UART_S1_RDRF_MASK) ? true : false;
}

void UART_PrintInt(int32_t value) {
    char buffer[12];  /* Max: "-2147483648\0" */
    char *ptr = buffer;
    uint32_t abs_value;
    
    /* Handle negative numbers */
    if (value < 0) {
        UART_TransmitByte('-');
        abs_value = (uint32_t)(-value);
    } else {
        abs_value = (uint32_t)value;
    }
    
    /* Convert to string (reverse order) */
    if (abs_value == 0) {
        UART_TransmitByte('0');
        return;
    }
    
    while (abs_value > 0) {
        *ptr++ = '0' + (abs_value % 10);
        abs_value /= 10;
    }
    
    /* Print in correct order */
    while (ptr > buffer) {
        UART_TransmitByte(*--ptr);
    }
}

void UART_PrintHex(uint32_t value) {
    const char hex_chars[] = "0123456789ABCDEF";
    char buffer[11];  /* "0x" + 8 hex digits + '\0' */
    
    buffer[0] = '0';
    buffer[1] = 'x';
    
    /* Convert to hex string */
    for (int i = 7; i >= 0; i--) {
        buffer[2 + i] = hex_chars[value & 0x0F];
        value >>= 4;
    }
    
    buffer[10] = '\0';
    UART_TransmitString(buffer);
}