/**
 * @file pwm.c
 * @brief PWM Driver Implementation for KL25Z
 * @details TPM0-based PWM generation for fan control
 */

#include "pwm.h"
#include "gpio.h"

/* System clock for TPM (48MHz from MCGFLLCLK) */
#define TPM_CLOCK_HZ            48000000u

/* Port D base for pin configuration */
#define PORTD_PCR0              (*(volatile uint32_t *)0x4004C000u)
#define PORT_PCR_MUX_ALT4       (4u << 8)

/*===========================================================================*/
/* Private Variables                                                         */
/*===========================================================================*/

static uint32_t pwm_mod_value = 0u;     /* Current MOD value */
static uint8_t  current_duty[6] = {0};  /* Duty cycle per channel */

/*===========================================================================*/
/* Public Function Implementations                                           */
/*===========================================================================*/

PWM_Status_t PWM_Init(const PWM_Config_t *config) {
    if (config == NULL) {
        return PWM_ERROR;
    }

    if (config->channel > 5u) {
        return PWM_INVALID_CHANNEL;
    }

    if (config->duty_cycle > 100u) {
        return PWM_INVALID_DUTY;
    }

    /* Enable clock to TPM0 */
    SIM_SCGC6_PWM |= SIM_SCGC6_TPM0_MSK;

    /* Select TPM clock source: MCGFLLCLK (48MHz) */
    SIM_SOPT2_PWM = (SIM_SOPT2_PWM & ~SIM_SOPT2_TPMSRC_MSK) | SIM_SOPT2_TPMSRC_MCGFLL;

    /* Enable clock to Port D for PWM pin */
    GPIO_EnablePortClock(3);  /* Port D */

    /* Configure PTD0 as TPM0_CH0 (ALT4) */
    PORTD_PCR0 = PORT_PCR_MUX_ALT4;

    /* Disable TPM0 during configuration */
    TPM0->SC = 0u;

    /* Calculate MOD value for desired frequency
     * PWM_freq = TPM_clock / (Prescaler * (MOD + 1))
     * MOD = (TPM_clock / (Prescaler * PWM_freq)) - 1
     *
     * For 25kHz with 48MHz clock and prescaler=1:
     * MOD = (48000000 / (1 * 25000)) - 1 = 1919
     */
    uint32_t prescaler = 1u;
    pwm_mod_value = (TPM_CLOCK_HZ / (prescaler * config->frequency_hz)) - 1u;

    /* Limit MOD to 16-bit value */
    if (pwm_mod_value > 0xFFFFu) {
        pwm_mod_value = 0xFFFFu;
    }

    /* Set modulo value */
    TPM0->MOD = pwm_mod_value;

    /* Configure channel for edge-aligned PWM, high-true pulses */
    TPM0->CHANNEL[config->channel].CnSC = TPM_CnSC_PWM_HIGH;

    /* Set initial duty cycle */
    uint32_t cv = (pwm_mod_value * config->duty_cycle) / 100u;
    TPM0->CHANNEL[config->channel].CnV = cv;
    current_duty[config->channel] = config->duty_cycle;

    /* Clear counter */
    TPM0->CNT = 0u;

    /* Configure and enable TPM0:
     * - Prescaler = 1 (PS = 0)
     * - Clock mode = TPM counter clock
     * - Edge-aligned mode (CPWMS = 0)
     */
    TPM0->SC = TPM_SC_CMOD_TPM_CLK | TPM_SC_PS_DIV1;

    return PWM_OK;
}

PWM_Status_t PWM_SetDutyCycle(uint8_t channel, uint8_t duty_percent) {
    if (channel > 5u) {
        return PWM_INVALID_CHANNEL;
    }

    if (duty_percent > 100u) {
        duty_percent = 100u;
    }

    /* Calculate channel value
     * CnV = (MOD * duty%) / 100
     */
    uint32_t cv = (pwm_mod_value * duty_percent) / 100u;
    TPM0->CHANNEL[channel].CnV = cv;
    current_duty[channel] = duty_percent;

    return PWM_OK;
}

uint8_t PWM_GetDutyCycle(uint8_t channel) {
    if (channel > 5u) {
        return 0u;
    }
    return current_duty[channel];
}

PWM_Status_t PWM_Start(void) {
    /* Enable TPM clock */
    TPM0->SC = (TPM0->SC & ~TPM_SC_CMOD_MASK) | TPM_SC_CMOD_TPM_CLK;
    return PWM_OK;
}

PWM_Status_t PWM_Stop(void) {
    /* Disable TPM clock */
    TPM0->SC &= ~TPM_SC_CMOD_MASK;
    return PWM_OK;
}

PWM_Status_t PWM_SetFrequency(uint32_t frequency_hz) {
    if (frequency_hz == 0u) {
        return PWM_ERROR;
    }

    /* Disable TPM during reconfiguration */
    uint32_t sc_backup = TPM0->SC;
    TPM0->SC = 0u;

    /* Calculate new MOD value */
    pwm_mod_value = (TPM_CLOCK_HZ / frequency_hz) - 1u;

    if (pwm_mod_value > 0xFFFFu) {
        pwm_mod_value = 0xFFFFu;
    }

    TPM0->MOD = pwm_mod_value;

    /* Update all channel values to maintain duty cycle */
    for (uint8_t ch = 0; ch < 6u; ch++) {
        uint32_t cv = (pwm_mod_value * current_duty[ch]) / 100u;
        TPM0->CHANNEL[ch].CnV = cv;
    }

    /* Clear counter and restore SC */
    TPM0->CNT = 0u;
    TPM0->SC = sc_backup;

    return PWM_OK;
}
