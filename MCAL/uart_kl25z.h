/**
 * @file uart_kl25z.h
 * @brief UART Driver for KL25Z - Debug Interface
 * @details Register-level UART0 implementation for KL25Z
 * 
 * Hardware Configuration:
 * - UART0 Module (connected to OpenSDA debug interface)
 * - PTA1: UART0_RX (Receive)
 * - PTA2: UART0_TX (Transmit)
 * - Baud Rate: 115200 (standard debug rate)
 * - Format: 8N1 (8 data bits, no parity, 1 stop bit)
 * 
 * Usage Example:
 * ```c
 * UART_Config_t config = {
 *     .baudrate = 115200,
 *     .data_bits = 8,
 *     .parity = UART_PARITY_NONE,
 *     .stop_bits = 1
 * };
 * UART_Init(&config);
 * UART_TransmitString("Hello BMS!\r\n");
 * ```
 */

#ifndef UART_KL25Z_H
#define UART_KL25Z_H

#include <stdint.h>
#include <stdbool.h>

/* UART0 Base Address */
#define UART0_BASE_PTR      ((UART_MemMapPtr)0x4006A000u)

/* UART Register Structure */
typedef struct {
    volatile uint8_t BDH;       /* Baud Rate High */
    volatile uint8_t BDL;       /* Baud Rate Low */
    volatile uint8_t C1;        /* Control Register 1 */
    volatile uint8_t C2;        /* Control Register 2 */
    volatile uint8_t S1;        /* Status Register 1 */
    volatile uint8_t S2;        /* Status Register 2 */
    volatile uint8_t C3;        /* Control Register 3 */
    volatile uint8_t D;         /* Data Register */
    volatile uint8_t MA1;       /* Match Address 1 */
    volatile uint8_t MA2;       /* Match Address 2 */
    volatile uint8_t C4;        /* Control Register 4 */
    volatile uint8_t C5;        /* Control Register 5 */
} UART_MemMap;

typedef UART_MemMap *UART_MemMapPtr;

/* UART Control Register 2 Bits */
#define UART_C2_TIE_MASK    0x80u   /* Transmit Interrupt Enable */
#define UART_C2_TCIE_MASK   0x40u   /* Transmission Complete Interrupt Enable */
#define UART_C2_RIE_MASK    0x20u   /* Receiver Interrupt Enable */
#define UART_C2_ILIE_MASK   0x10u   /* Idle Line Interrupt Enable */
#define UART_C2_TE_MASK     0x08u   /* Transmitter Enable */
#define UART_C2_RE_MASK     0x04u   /* Receiver Enable */

/* UART Status Register 1 Bits */
#define UART_S1_TDRE_MASK   0x80u   /* Transmit Data Register Empty */
#define UART_S1_TC_MASK     0x40u   /* Transmission Complete */
#define UART_S1_RDRF_MASK   0x20u   /* Receive Data Register Full */
#define UART_S1_IDLE_MASK   0x10u   /* Idle Line Flag */
#define UART_S1_OR_MASK     0x08u   /* Receiver Overrun */
#define UART_S1_NF_MASK     0x04u   /* Noise Flag */
#define UART_S1_FE_MASK     0x02u   /* Framing Error */
#define UART_S1_PF_MASK     0x01u   /* Parity Error */

/* Clock Gate Control */
#define SIM_SCGC4_UART0_MASK 0x00000400u
#define SIM_SCGC5_PORTA_MASK 0x00000200u

/* Port Control for UART Pins */
#define PORTA_BASE          0x40049000u
#define PORTA_PCR1          (*(volatile uint32_t *)(PORTA_BASE + 0x04))  /* RX */
#define PORTA_PCR2          (*(volatile uint32_t *)(PORTA_BASE + 0x08))  /* TX */

/* UART Configuration Constants */
#define UART_DEFAULT_CLOCK  48000000u   /* 48MHz system clock (default PLL) */
#define UART_CLOCK_DIV      16u         /* UART uses clock/16 for oversampling */

/* Parity options */
typedef enum {
    UART_PARITY_NONE = 0,
    UART_PARITY_EVEN = 1,
    UART_PARITY_ODD = 2
} UART_Parity_t;

/* Status codes */
typedef enum {
    UART_OK = 0,
    UART_ERROR = 1,
    UART_TIMEOUT = 2,
    UART_BUSY = 3
} UART_Status_t;

/* UART Configuration Structure */
typedef struct {
    uint32_t baudrate;          /* Baud rate in bps (e.g., 115200) */
    uint8_t  data_bits;         /* 8 or 9 */
    UART_Parity_t parity;       /* None, Even, or Odd */
    uint8_t  stop_bits;         /* 1 or 2 */
} UART_Config_t;

/* Function Prototypes */

/**
 * @brief Initialize UART0 peripheral
 * @param config Pointer to UART configuration
 * @return UART_OK on success
 * 
 * Configures UART0 with specified parameters. After initialization,
 * UART0_TX (PTA2) and UART0_RX (PTA1) are ready for communication.
 */
UART_Status_t UART_Init(const UART_Config_t *config);

/**
 * @brief Transmit single byte (blocking)
 * @param data Byte to transmit
 * @return UART_OK on success
 * 
 * Waits for transmit buffer to be empty, then sends byte.
 * Includes timeout protection.
 */
UART_Status_t UART_TransmitByte(uint8_t data);

/**
 * @brief Transmit null-terminated string
 * @param str Pointer to string
 * @return UART_OK on success
 * 
 * Sends each character until null terminator.
 * Example: UART_TransmitString("Voltage: 3850mV\r\n");
 */
UART_Status_t UART_TransmitString(const char *str);

/**
 * @brief Transmit buffer of specific length
 * @param data Pointer to data buffer
 * @param length Number of bytes to send
 * @return UART_OK on success
 */
UART_Status_t UART_TransmitBuffer(const uint8_t *data, uint16_t length);

/**
 * @brief Receive single byte (blocking with timeout)
 * @param data Pointer to store received byte
 * @param timeout_ms Timeout in milliseconds (0 = no timeout)
 * @return UART_OK on success, UART_TIMEOUT on timeout
 */
UART_Status_t UART_ReceiveByte(uint8_t *data, uint32_t timeout_ms);

/**
 * @brief Check if data is available to receive
 * @return true if data available, false otherwise
 */
bool UART_DataAvailable(void);

/**
 * @brief Print formatted integer (decimal)
 * @param value Integer value to print
 * 
 * Helper function for printing numbers.
 * Example: UART_PrintInt(3850) outputs "3850"
 */
void UART_PrintInt(int32_t value);

/**
 * @brief Print formatted hex value
 * @param value Value to print in hexadecimal
 * 
 * Example: UART_PrintHex(0x1A) outputs "0x1A"
 */
void UART_PrintHex(uint32_t value);

#endif /* UART_KL25Z_H */