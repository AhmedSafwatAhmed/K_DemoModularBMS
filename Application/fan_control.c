#include "fan_control.h"
#include "pwm.h"
#include "bms_config.h"

static uint8_t current_fan_speed = 0;
static bool fan_active = false;

void FanControl_Init(void) {
    PWM_Config_t pwm_config = {
        .frequency_hz = BMS_FAN_PWM_FREQ_HZ,
        .duty_cycle = 0,
        .channel = PWM_TIMER_CHANNEL
    };
    PWM_Init(&pwm_config);
}

void FanControl_Update(int8_t temperature_C) {
    /* Hysteresis control */
    if (temperature_C >= BMS_TEMP_FAN_ON && !fan_active) {
        fan_active = true;
    } else if (temperature_C <= BMS_TEMP_FAN_OFF && fan_active) {
        fan_active = false;
    }
    
    if (!fan_active) {
        FanControl_SetSpeed(0);
        return;
    }
    
    /* Linear mapping: 40°C=30%, 60°C=100% */
    if (temperature_C >= 60) {
        FanControl_SetSpeed(100);
    } else if (temperature_C <= 40) {
        FanControl_SetSpeed(BMS_FAN_MIN_DUTY);
    } else {
        uint8_t speed = BMS_FAN_MIN_DUTY + 
                        ((temperature_C - 40) * (100 - BMS_FAN_MIN_DUTY)) / 20;
        FanControl_SetSpeed(speed);
    }
}

void FanControl_SetSpeed(uint8_t speed_percent) {
    if (speed_percent > 100) speed_percent = 100;
    current_fan_speed = speed_percent;
    PWM_SetDutyCycle(PWM_TIMER_CHANNEL, speed_percent);
}

uint8_t FanControl_GetSpeed(void) {
    return current_fan_speed;
}