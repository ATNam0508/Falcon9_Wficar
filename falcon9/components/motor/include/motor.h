#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct motor_handler motor_handler_t;

/**
 * @brief Create a new motor handler
 * 
 * @param dir_gpio GPIO pin used for direction control
 * @param pwm_gpio GPIO pin used for PWM signal output
 * @param pwm_channel PWM channel (0–7), each motor should use a separate channel
 * @param pwm_timer PWM timer to use (LED_TIMER_0 ~ LED_TIMER_3)
 * @return motor_handler_t* Pointer to the new motor handler
 */
motor_handler_t *motor_new_handle(gpio_num_t dir_gpio, gpio_num_t pwm_gpio,
                                  ledc_channel_t pwm_channel, ledc_timer_t pwm_timer);

/**
 * @brief Free the motor handler
 * 
 * @param self Pointer to the motor handler
 */
void motor_delete_handle(motor_handler_t *self);

/**
 * @brief Enable or disable the motor
 * 
 * @param self Pointer to the motor handler
 * @param enable true to enable, false to disable
 */
void motor_set_enable(motor_handler_t *self, bool enable);

/**
 * @brief Set the motor rotation direction
 * 
 * @param self Pointer to the motor handler
 * @param forward true for forward direction, false for reverse
 */
void motor_set_direction(motor_handler_t *self, bool forward);

/**
 * @brief Set motor speed using duty cycle (12-bit resolution)
 * 
 * @param self Pointer to the motor handler
 * @param duty_cycle Duty cycle value (0–4095)
 */
void motor_set_speed(motor_handler_t *self, uint16_t duty_cycle);

#ifdef __cplusplus
}
#endif
