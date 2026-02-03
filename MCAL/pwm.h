/**
 * @file pwm.h
 * @brief PWM Driver for KL25Z using TPM (Timer/PWM Module)
 * @details TPM-based PWM generation for fan control
 *
 * KL25Z has three TPM modules:
 * - TPM0: 6 channels
 * - TPM1: 2 channels
 * - TPM2: 2 channels
 *
 * PWM Configuration for Fan Control:
 * - TPM0, Channel 0 (PTD0, ALT4)
 * - Frequency: 25kHz (typical for DC fans)
 * - Resolution: ~1920 steps at 48MHz clock
 */

#ifndef PWM_H
#define PWM_H

#include <stdint.h>
#include <stdbool.h>

/*===========================================================================*/
/* TPM Register Definitions                                                  */
/*===========================================================================*/

typedef struct {
    volatile uint32_t SC;       /* Status and Control */
    volatile uint32_t CNT;      /* Counter */
    volatile uint32_t MOD;      /* Modulo */
    struct {
        volatile uint32_t CnSC; /* Channel Status and Control */
        volatile uint32_t CnV;  /* Channel Value */
    } CHANNEL[6];
    uint32_t RESERVED0[5];
    volatile uint32_t STATUS;   /* Capture and Compare Status */
    uint32_t RESERVED1[12];
    volatile uint32_t CONF;     /* Configuration */
} TPM_Type;

#define TPM0                    ((TPM_Type *)0x40038000u)
#define TPM1                    ((TPM_Type *)0x40039000u)
#define TPM2                    ((TPM_Type *)0x4003A000u)

/*===========================================================================*/
/* TPM Register Bit Definitions                                              */
/*===========================================================================*/

/* Status and Control Register (TPMx_SC) */
#define TPM_SC_PS_MASK          (7u << 0)   /* Prescale Factor */
#define TPM_SC_PS_DIV1          (0u << 0)
#define TPM_SC_PS_DIV2          (1u << 0)
#define TPM_SC_PS_DIV4          (2u << 0)
#define TPM_SC_PS_DIV8          (3u << 0)
#define TPM_SC_PS_DIV16         (4u << 0)
#define TPM_SC_PS_DIV32         (5u << 0)
#define TPM_SC_PS_DIV64         (6u << 0)
#define TPM_SC_PS_DIV128        (7u << 0)
#define TPM_SC_CMOD_MASK        (3u << 3)   /* Clock Mode */
#define TPM_SC_CMOD_DISABLED    (0u << 3)
#define TPM_SC_CMOD_TPM_CLK     (1u << 3)   /* TPM counter clock */
#define TPM_SC_CMOD_EXT_CLK     (2u << 3)   /* External clock */
#define TPM_SC_CPWMS_MASK       (1u << 5)   /* Center-aligned PWM */
#define TPM_SC_TOIE_MASK        (1u << 6)   /* Timer Overflow Int Enable */
#define TPM_SC_TOF_MASK         (1u << 7)   /* Timer Overflow Flag */

/* Channel Status and Control Register (TPMx_CnSC) */
#define TPM_CnSC_DMA_MASK       (1u << 0)   /* DMA Enable */
#define TPM_CnSC_ELSA_MASK      (1u << 2)   /* Edge/Level Select A */
#define TPM_CnSC_ELSB_MASK      (1u << 3)   /* Edge/Level Select B */
#define TPM_CnSC_MSA_MASK       (1u << 4)   /* Mode Select A */
#define TPM_CnSC_MSB_MASK       (1u << 5)   /* Mode Select B */
#define TPM_CnSC_CHIE_MASK      (1u << 6)   /* Channel Int Enable */
#define TPM_CnSC_CHF_MASK       (1u << 7)   /* Channel Flag */

/* Edge-aligned PWM: MSB=1, MSA=0, ELSB=1, ELSA=0 (high-true) */
#define TPM_CnSC_PWM_HIGH       (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK)
/* Edge-aligned PWM: MSB=1, MSA=0, ELSB=0, ELSA=1 (low-true) */
#define TPM_CnSC_PWM_LOW        (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK)

/*===========================================================================*/
/* Clock Gating                                                              */
/*===========================================================================*/

#define SIM_SCGC6_PWM           (*(volatile uint32_t *)0x4004803Cu)
#define SIM_SCGC6_TPM0_MSK      (1u << 24)
#define SIM_SCGC6_TPM1_MSK      (1u << 25)
#define SIM_SCGC6_TPM2_MSK      (1u << 26)

#define SIM_SOPT2_PWM           (*(volatile uint32_t *)0x40048004u)
#define SIM_SOPT2_TPMSRC_MSK    (3u << 24)
#define SIM_SOPT2_TPMSRC_MCGFLL (1u << 24)  /* MCGFLLCLK or MCGPLLCLK/2 */

/*===========================================================================*/
/* PWM Configuration Types                                                   */
/*===========================================================================*/

typedef enum {
    PWM_OK = 0,
    PWM_ERROR = 1,
    PWM_INVALID_CHANNEL = 2,
    PWM_INVALID_DUTY = 3
} PWM_Status_t;

typedef struct {
    uint32_t frequency_hz;      /* PWM frequency in Hz */
    uint8_t  duty_cycle;        /* Initial duty cycle (0-100%) */
    uint8_t  channel;           /* TPM channel number */
} PWM_Config_t;

/*===========================================================================*/
/* Function Prototypes                                                       */
/*===========================================================================*/

/**
 * @brief Initialize PWM on TPM0 for fan control
 * @param config Pointer to PWM configuration
 * @return PWM_OK on success
 *
 * Configures TPM0 for edge-aligned PWM mode with specified frequency.
 * Pin configuration (PTD0 = TPM0_CH0) must be done separately.
 */
PWM_Status_t PWM_Init(const PWM_Config_t *config);

/**
 * @brief Set PWM duty cycle
 * @param channel TPM channel number (0-5 for TPM0)
 * @param duty_percent Duty cycle percentage (0-100)
 * @return PWM_OK on success
 *
 * 0% = always low, 100% = always high
 */
PWM_Status_t PWM_SetDutyCycle(uint8_t channel, uint8_t duty_percent);

/**
 * @brief Get current PWM duty cycle
 * @param channel TPM channel number
 * @return Current duty cycle percentage (0-100)
 */
uint8_t PWM_GetDutyCycle(uint8_t channel);

/**
 * @brief Start PWM generation
 * @return PWM_OK on success
 */
PWM_Status_t PWM_Start(void);

/**
 * @brief Stop PWM generation
 * @return PWM_OK on success
 */
PWM_Status_t PWM_Stop(void);

/**
 * @brief Set PWM frequency (requires re-initialization)
 * @param frequency_hz New frequency in Hz
 * @return PWM_OK on success
 */
PWM_Status_t PWM_SetFrequency(uint32_t frequency_hz);

#endif /* PWM_H */
