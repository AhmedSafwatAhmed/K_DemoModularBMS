/**
 * @file gpio.c
 * @brief GPIO Driver Implementation for KL25Z
 * @details Register-level implementation following KL25 Reference Manual
 */

#include "gpio.h"

/*===========================================================================*/
/* Private Helper Functions                                                  */
/*===========================================================================*/

/**
 * @brief Get GPIO peripheral pointer from port number
 */
static GPIO_Type* get_gpio_base(uint8_t port) {
    static GPIO_Type* const gpio_bases[] = {
        GPIOA, GPIOB, GPIOC, GPIOD, GPIOE
    };

    if (port > 4u) {
        return (GPIO_Type*)0;
    }
    return gpio_bases[port];
}

/**
 * @brief Get PORT peripheral pointer from port number
 */
static PORT_Type* get_port_base(uint8_t port) {
    static PORT_Type* const port_bases[] = {
        PORTA, PORTB, PORTC, PORTD, PORTE
    };

    if (port > 4u) {
        return (PORT_Type*)0;
    }
    return port_bases[port];
}

/*===========================================================================*/
/* Public Function Implementations                                           */
/*===========================================================================*/

void GPIO_EnablePortClock(uint8_t port) {
    static const uint32_t port_clock_masks[] = {
        SIM_SCGC5_PORTA_MSK,
        SIM_SCGC5_PORTB_MSK,
        SIM_SCGC5_PORTC_MSK,
        SIM_SCGC5_PORTD_MSK,
        SIM_SCGC5_PORTE_MSK
    };

    if (port <= 4u) {
        SIM_SCGC5_GPIO |= port_clock_masks[port];
    }
}

GPIO_Status_t GPIO_Init(uint8_t port, uint8_t pin, GPIO_Direction_t direction) {
    if (port > 4u) {
        return GPIO_INVALID_PORT;
    }
    if (pin > 31u) {
        return GPIO_INVALID_PIN;
    }

    GPIO_Type* gpio = get_gpio_base(port);
    PORT_Type* port_ctrl = get_port_base(port);

    if (gpio == (GPIO_Type*)0 || port_ctrl == (PORT_Type*)0) {
        return GPIO_ERROR;
    }

    /* Enable clock for this port */
    GPIO_EnablePortClock(port);

    /* Configure pin as GPIO (MUX = 1) */
    port_ctrl->PCR[pin] = (port_ctrl->PCR[pin] & ~PORT_PCR_MUX_MASK) |
                          (GPIO_MUX_GPIO << PORT_PCR_MUX_SHIFT);

    /* Set direction */
    if (direction == GPIO_DIR_OUTPUT) {
        gpio->PDDR |= (1u << pin);
    } else {
        gpio->PDDR &= ~(1u << pin);
    }

    return GPIO_OK;
}

GPIO_Status_t GPIO_ConfigurePull(uint8_t port, uint8_t pin, GPIO_Pull_t pull) {
    if (port > 4u) {
        return GPIO_INVALID_PORT;
    }
    if (pin > 31u) {
        return GPIO_INVALID_PIN;
    }

    PORT_Type* port_ctrl = get_port_base(port);
    if (port_ctrl == (PORT_Type*)0) {
        return GPIO_ERROR;
    }

    /* Clear pull configuration */
    port_ctrl->PCR[pin] &= ~(PORT_PCR_PE_MASK | PORT_PCR_PS_MASK);

    switch (pull) {
        case GPIO_PULL_UP:
            port_ctrl->PCR[pin] |= (PORT_PCR_PE_MASK | PORT_PCR_PS_MASK);
            break;
        case GPIO_PULL_DOWN:
            port_ctrl->PCR[pin] |= PORT_PCR_PE_MASK;
            break;
        case GPIO_PULL_NONE:
        default:
            /* Pull already disabled */
            break;
    }

    return GPIO_OK;
}

void GPIO_SetBit(uint8_t port, uint8_t pin) {
    GPIO_Type* gpio = get_gpio_base(port);
    if (gpio != (GPIO_Type*)0 && pin <= 31u) {
        gpio->PSOR = (1u << pin);  /* Write to PSOR sets the pin */
    }
}

void GPIO_ClearBit(uint8_t port, uint8_t pin) {
    GPIO_Type* gpio = get_gpio_base(port);
    if (gpio != (GPIO_Type*)0 && pin <= 31u) {
        gpio->PCOR = (1u << pin);  /* Write to PCOR clears the pin */
    }
}

void GPIO_ToggleBit(uint8_t port, uint8_t pin) {
    GPIO_Type* gpio = get_gpio_base(port);
    if (gpio != (GPIO_Type*)0 && pin <= 31u) {
        gpio->PTOR = (1u << pin);  /* Write to PTOR toggles the pin */
    }
}

bool GPIO_ReadBit(uint8_t port, uint8_t pin) {
    GPIO_Type* gpio = get_gpio_base(port);
    if (gpio != (GPIO_Type*)0 && pin <= 31u) {
        return (gpio->PDIR & (1u << pin)) != 0u;
    }
    return false;
}

void GPIO_WriteBit(uint8_t port, uint8_t pin, bool value) {
    if (value) {
        GPIO_SetBit(port, pin);
    } else {
        GPIO_ClearBit(port, pin);
    }
}

void GPIO_SetDirection(uint8_t port, uint8_t pin, GPIO_Direction_t direction) {
    GPIO_Type* gpio = get_gpio_base(port);
    if (gpio != (GPIO_Type*)0 && pin <= 31u) {
        if (direction == GPIO_DIR_OUTPUT) {
            gpio->PDDR |= (1u << pin);
        } else {
            gpio->PDDR &= ~(1u << pin);
        }
    }
}

GPIO_Status_t GPIO_SetMux(uint8_t port, uint8_t pin, uint8_t mux) {
    if (port > 4u) {
        return GPIO_INVALID_PORT;
    }
    if (pin > 31u) {
        return GPIO_INVALID_PIN;
    }
    if (mux > 7u) {
        return GPIO_ERROR;
    }

    PORT_Type* port_ctrl = get_port_base(port);
    if (port_ctrl == (PORT_Type*)0) {
        return GPIO_ERROR;
    }

    /* Enable clock for this port */
    GPIO_EnablePortClock(port);

    /* Configure MUX field */
    port_ctrl->PCR[pin] = (port_ctrl->PCR[pin] & ~PORT_PCR_MUX_MASK) |
                          ((uint32_t)mux << PORT_PCR_MUX_SHIFT);

    return GPIO_OK;
}
