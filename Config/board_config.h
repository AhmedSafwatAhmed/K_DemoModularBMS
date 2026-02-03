/**
 * @file board_config.h
 * @brief Board-Specific Hardware Configuration for FRDM-KL25Z
 * @details Pin assignments and hardware constants for NXP FRDM-KL25Z board
 *
 * Board: FRDM-KL25Z (MKL25Z128VLK4)
 * - ARM Cortex-M0+ @ 48MHz
 * - 128KB Flash, 16KB SRAM
 * - OpenSDA debug interface
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/*===========================================================================*/
/* System Clock Configuration                                                */
/*===========================================================================*/

/** Core/System clock frequency (48MHz with PLL) */
#define SYSTEM_CLOCK_HZ             48000000u

/** Bus clock frequency (24MHz, half of system clock) */
#define BUS_CLOCK_HZ                24000000u

/** SysTick reload value for 1ms tick (using core clock) */
#define SYSTICK_RELOAD_1MS          (SYSTEM_CLOCK_HZ / 1000u - 1u)

/*===========================================================================*/
/* LED Pin Configuration (On-board RGB LED)                                  */
/*===========================================================================*/

/** Red LED: PTB18 (active low) */
#define LED_RED_PORT                1u      /* Port B */
#define LED_RED_PIN                 18u

/** Green LED: PTB19 (active low) */
#define LED_GREEN_PORT              1u      /* Port B */
#define LED_GREEN_PIN               19u

/** Blue LED: PTD1 (active low) */
#define LED_BLUE_PORT               3u      /* Port D */
#define LED_BLUE_PIN                1u

/*===========================================================================*/
/* SPI Configuration for MC33664B Transceiver                               */
/*===========================================================================*/

/** SPI0 Module - Port C pins */
#define SPI_PORT                    2u      /* Port C */

/** SPI0_SCK: PTC5 (ALT2) */
#define SPI_SCK_PIN                 5u

/** SPI0_MOSI: PTC6 (ALT2) */
#define SPI_MOSI_PIN                6u

/** SPI0_MISO: PTC7 (ALT2) */
#define SPI_MISO_PIN                7u

/** SPI0_CS: PTC4 (GPIO controlled) */
#define SPI_CS_PIN                  4u

/** MC33664B Enable/Disable pin: PTC3 (optional) */
#define MC33664_EN_PIN              3u

/** MC33664B Interrupt pin: PTC0 (optional) */
#define MC33664_INT_PIN             0u

/*===========================================================================*/
/* UART Configuration for Debug Interface                                    */
/*===========================================================================*/

/** UART0 Module - Port A pins (connected to OpenSDA) */
#define DEBUG_UART_PORT             0u      /* Port A */

/** UART0_RX: PTA1 (ALT2) */
#define DEBUG_UART_RX_PIN           1u

/** UART0_TX: PTA2 (ALT2) */
#define DEBUG_UART_TX_PIN           2u

/*===========================================================================*/
/* I2C Configuration for LCD Display                                         */
/*===========================================================================*/

/** I2C0 Module - Port E pins */
#define I2C_PORT                    4u      /* Port E */

/** I2C0_SCL: PTE24 (ALT5) */
#define I2C_SCL_PIN                 24u

/** I2C0_SDA: PTE25 (ALT5) */
#define I2C_SDA_PIN                 25u

/*===========================================================================*/
/* PWM Configuration for Fan Control                                         */
/*===========================================================================*/

/** TPM0 Module - Port D pin for fan PWM */
#define FAN_PWM_PORT                3u      /* Port D */

/** TPM0_CH0: PTD0 (ALT4) - Fan PWM output */
#define FAN_PWM_PIN                 0u

/** TPM module used for PWM */
#define FAN_TPM_MODULE              0u

/** TPM channel for fan PWM */
#define FAN_TPM_CHANNEL             0u

/*===========================================================================*/
/* GPIO Base Addresses                                                       */
/*===========================================================================*/

#define GPIOA_BASE                  0x400FF000u
#define GPIOB_BASE                  0x400FF040u
#define GPIOC_BASE                  0x400FF080u
#define GPIOD_BASE                  0x400FF0C0u
#define GPIOE_BASE                  0x400FF100u

/*===========================================================================*/
/* Port Control Module Base Addresses                                        */
/*===========================================================================*/

#define PORTA_PCR_BASE              0x40049000u
#define PORTB_PCR_BASE              0x4004A000u
#define PORTC_PCR_BASE              0x4004B000u
#define PORTD_PCR_BASE              0x4004C000u
#define PORTE_PCR_BASE              0x4004D000u

/*===========================================================================*/
/* System Integration Module (SIM) Registers                                 */
/*===========================================================================*/

#define SIM_BASE_ADDR               0x40047000u

/** System Clock Gating Control Register 4 */
#define SIM_SCGC4_REG               (*(volatile uint32_t *)(SIM_BASE_ADDR + 0x1034u))
#define SIM_SCGC4_I2C0_MASK         (1u << 6)
#define SIM_SCGC4_I2C1_MASK         (1u << 7)
#define SIM_SCGC4_UART0_MASK        (1u << 10)
#define SIM_SCGC4_UART1_MASK        (1u << 11)
#define SIM_SCGC4_UART2_MASK        (1u << 12)
#define SIM_SCGC4_SPI0_MASK_CFG     (1u << 22)
#define SIM_SCGC4_SPI1_MASK         (1u << 23)

/** System Clock Gating Control Register 5 */
#define SIM_SCGC5_REG               (*(volatile uint32_t *)(SIM_BASE_ADDR + 0x1038u))
#define SIM_SCGC5_PORTA_MASK        (1u << 9)
#define SIM_SCGC5_PORTB_MASK        (1u << 10)
#define SIM_SCGC5_PORTC_MASK_CFG    (1u << 11)
#define SIM_SCGC5_PORTD_MASK        (1u << 12)
#define SIM_SCGC5_PORTE_MASK        (1u << 13)

/** System Clock Gating Control Register 6 */
#define SIM_SCGC6_REG               (*(volatile uint32_t *)(SIM_BASE_ADDR + 0x103Cu))
#define SIM_SCGC6_TPM0_MASK         (1u << 24)
#define SIM_SCGC6_TPM1_MASK         (1u << 25)
#define SIM_SCGC6_TPM2_MASK         (1u << 26)
#define SIM_SCGC6_ADC0_MASK         (1u << 27)

/** System Options Register 2 - TPM Clock Source */
#define SIM_SOPT2_REG               (*(volatile uint32_t *)(SIM_BASE_ADDR + 0x1004u))
#define SIM_SOPT2_TPMSRC_MASK       (3u << 24)
#define SIM_SOPT2_TPMSRC_MCGFLLCLK  (1u << 24)
#define SIM_SOPT2_UART0SRC_MASK     (3u << 26)
#define SIM_SOPT2_UART0SRC_MCGFLLCLK (1u << 26)

/*===========================================================================*/
/* Port MUX Configuration Values                                             */
/*===========================================================================*/

#define PORT_PCR_MUX_DISABLED       (0u << 8)
#define PORT_PCR_MUX_GPIO           (1u << 8)
#define PORT_PCR_MUX_ALT2           (2u << 8)
#define PORT_PCR_MUX_ALT3           (3u << 8)
#define PORT_PCR_MUX_ALT4           (4u << 8)
#define PORT_PCR_MUX_ALT5           (5u << 8)

/** Pull-up/Pull-down configuration */
#define PORT_PCR_PE_MASK            (1u << 1)
#define PORT_PCR_PS_MASK            (1u << 0)
#define PORT_PCR_PULLUP             (PORT_PCR_PE_MASK | PORT_PCR_PS_MASK)
#define PORT_PCR_PULLDOWN           (PORT_PCR_PE_MASK)

/*===========================================================================*/
/* Contactor Control Pins (External relay/contactor)                         */
/*===========================================================================*/

/** Main contactor control: PTE20 */
#define CONTACTOR_MAIN_PORT         4u      /* Port E */
#define CONTACTOR_MAIN_PIN          20u

/** Precharge contactor control: PTE21 */
#define CONTACTOR_PRECHARGE_PORT    4u      /* Port E */
#define CONTACTOR_PRECHARGE_PIN     21u

/** Contactor feedback input: PTE22 */
#define CONTACTOR_FEEDBACK_PORT     4u      /* Port E */
#define CONTACTOR_FEEDBACK_PIN      22u

/*===========================================================================*/
/* Safety Output Pins                                                        */
/*===========================================================================*/

/** Emergency shutdown output: PTE23 */
#define SAFETY_SHUTDOWN_PORT        4u      /* Port E */
#define SAFETY_SHUTDOWN_PIN         23u

/** Fault indicator output: PTB0 */
#define FAULT_INDICATOR_PORT        1u      /* Port B */
#define FAULT_INDICATOR_PIN         0u

#endif /* BOARD_CONFIG_H */
