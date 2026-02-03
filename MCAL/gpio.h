/**
 * @file gpio.h
 * @brief GPIO Driver for KL25Z
 * @details Register-level GPIO implementation for KL25Z
 *
 * GPIO Ports on KL25Z:
 * - Port A: GPIOA (0x400FF000)
 * - Port B: GPIOB (0x400FF040)
 * - Port C: GPIOC (0x400FF080)
 * - Port D: GPIOD (0x400FF0C0)
 * - Port E: GPIOE (0x400FF100)
 *
 * Each port has:
 * - PDOR: Port Data Output Register
 * - PSOR: Port Set Output Register
 * - PCOR: Port Clear Output Register
 * - PTOR: Port Toggle Output Register
 * - PDIR: Port Data Input Register
 * - PDDR: Port Data Direction Register
 */

#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include <stdbool.h>

/*===========================================================================*/
/* GPIO Register Structure                                                   */
/*===========================================================================*/

typedef struct {
    volatile uint32_t PDOR;     /* Port Data Output Register */
    volatile uint32_t PSOR;     /* Port Set Output Register */
    volatile uint32_t PCOR;     /* Port Clear Output Register */
    volatile uint32_t PTOR;     /* Port Toggle Output Register */
    volatile uint32_t PDIR;     /* Port Data Input Register */
    volatile uint32_t PDDR;     /* Port Data Direction Register */
} GPIO_Type;

/*===========================================================================*/
/* GPIO Base Addresses                                                       */
/*===========================================================================*/

#define GPIOA                   ((GPIO_Type *)0x400FF000u)
#define GPIOB                   ((GPIO_Type *)0x400FF040u)
#define GPIOC                   ((GPIO_Type *)0x400FF080u)
#define GPIOD                   ((GPIO_Type *)0x400FF0C0u)
#define GPIOE                   ((GPIO_Type *)0x400FF100u)

/*===========================================================================*/
/* Port Control Register Structure                                           */
/*===========================================================================*/

typedef struct {
    volatile uint32_t PCR[32];  /* Pin Control Registers */
    volatile uint32_t GPCLR;    /* Global Pin Control Low Register */
    volatile uint32_t GPCHR;    /* Global Pin Control High Register */
    uint32_t RESERVED[6];
    volatile uint32_t ISFR;     /* Interrupt Status Flag Register */
} PORT_Type;

#define PORTA                   ((PORT_Type *)0x40049000u)
#define PORTB                   ((PORT_Type *)0x4004A000u)
#define PORTC                   ((PORT_Type *)0x4004B000u)
#define PORTD                   ((PORT_Type *)0x4004C000u)
#define PORTE                   ((PORT_Type *)0x4004D000u)

/*===========================================================================*/
/* Pin Control Register Bit Definitions                                      */
/*===========================================================================*/

#define PORT_PCR_PS_MASK        (1u << 0)   /* Pull Select */
#define PORT_PCR_PE_MASK        (1u << 1)   /* Pull Enable */
#define PORT_PCR_SRE_MASK       (1u << 2)   /* Slew Rate Enable */
#define PORT_PCR_PFE_MASK       (1u << 4)   /* Passive Filter Enable */
#define PORT_PCR_DSE_MASK       (1u << 6)   /* Drive Strength Enable */
#define PORT_PCR_MUX_MASK       (7u << 8)   /* Pin Mux Control */
#define PORT_PCR_MUX_SHIFT      8u
#define PORT_PCR_IRQC_MASK      (0xFu << 16) /* Interrupt Config */
#define PORT_PCR_ISF_MASK       (1u << 24)  /* Interrupt Status Flag */

/* MUX Field Values */
#define GPIO_MUX_DISABLED       0u
#define GPIO_MUX_GPIO           1u
#define GPIO_MUX_ALT2           2u
#define GPIO_MUX_ALT3           3u
#define GPIO_MUX_ALT4           4u
#define GPIO_MUX_ALT5           5u
#define GPIO_MUX_ALT6           6u
#define GPIO_MUX_ALT7           7u

/*===========================================================================*/
/* Clock Gating                                                              */
/*===========================================================================*/

#define SIM_SCGC5_GPIO          (*(volatile uint32_t *)0x40048038u)
#define SIM_SCGC5_PORTA_MSK     (1u << 9)
#define SIM_SCGC5_PORTB_MSK     (1u << 10)
#define SIM_SCGC5_PORTC_MSK     (1u << 11)
#define SIM_SCGC5_PORTD_MSK     (1u << 12)
#define SIM_SCGC5_PORTE_MSK     (1u << 13)

/*===========================================================================*/
/* GPIO Direction Enumerations                                               */
/*===========================================================================*/

typedef enum {
    GPIO_DIR_INPUT = 0,
    GPIO_DIR_OUTPUT = 1
} GPIO_Direction_t;

typedef enum {
    GPIO_PULL_NONE = 0,
    GPIO_PULL_DOWN = 1,
    GPIO_PULL_UP = 2
} GPIO_Pull_t;

typedef enum {
    GPIO_OK = 0,
    GPIO_ERROR = 1,
    GPIO_INVALID_PORT = 2,
    GPIO_INVALID_PIN = 3
} GPIO_Status_t;

/*===========================================================================*/
/* Function Prototypes                                                       */
/*===========================================================================*/

/**
 * @brief Initialize a GPIO pin
 * @param port Port number (0=A, 1=B, 2=C, 3=D, 4=E)
 * @param pin Pin number (0-31)
 * @param direction Input or Output
 * @return GPIO_OK on success
 */
GPIO_Status_t GPIO_Init(uint8_t port, uint8_t pin, GPIO_Direction_t direction);

/**
 * @brief Configure pin with pull-up/pull-down
 * @param port Port number
 * @param pin Pin number
 * @param pull Pull configuration
 * @return GPIO_OK on success
 */
GPIO_Status_t GPIO_ConfigurePull(uint8_t port, uint8_t pin, GPIO_Pull_t pull);

/**
 * @brief Set a GPIO pin high
 * @param port Port number
 * @param pin Pin number
 */
void GPIO_SetBit(uint8_t port, uint8_t pin);

/**
 * @brief Set a GPIO pin low
 * @param port Port number
 * @param pin Pin number
 */
void GPIO_ClearBit(uint8_t port, uint8_t pin);

/**
 * @brief Toggle a GPIO pin
 * @param port Port number
 * @param pin Pin number
 */
void GPIO_ToggleBit(uint8_t port, uint8_t pin);

/**
 * @brief Read a GPIO pin state
 * @param port Port number
 * @param pin Pin number
 * @return true if pin is high, false if low
 */
bool GPIO_ReadBit(uint8_t port, uint8_t pin);

/**
 * @brief Write a value to a GPIO pin
 * @param port Port number
 * @param pin Pin number
 * @param value true for high, false for low
 */
void GPIO_WriteBit(uint8_t port, uint8_t pin, bool value);

/**
 * @brief Set pin direction
 * @param port Port number
 * @param pin Pin number
 * @param direction Input or Output
 */
void GPIO_SetDirection(uint8_t port, uint8_t pin, GPIO_Direction_t direction);

/**
 * @brief Configure pin alternate function
 * @param port Port number
 * @param pin Pin number
 * @param mux Alternate function (0-7)
 * @return GPIO_OK on success
 */
GPIO_Status_t GPIO_SetMux(uint8_t port, uint8_t pin, uint8_t mux);

/**
 * @brief Enable clock for a GPIO port
 * @param port Port number
 */
void GPIO_EnablePortClock(uint8_t port);

#endif /* GPIO_H */
