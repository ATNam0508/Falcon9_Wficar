#include "motor.h"
#include <stdlib.h>

/**
 * @brief Structure that holds motor control configuration
 */
struct motor_handler {
    gpio_num_t dir_pin;           ///< GPIO pin for motor direction control
    gpio_num_t pwm_pin;           ///< GPIO pin for PWM signal output
    ledc_channel_t pwm_channel;   ///< LEDC PWM channel used
};

/**
 * @brief Initialize a new motor handler
 * 
 * @param dir_gpio GPIO pin for controlling direction
 * @param pwm_gpio GPIO pin for PWM output
 * @param pwm_channel LEDC channel to be used for PWM
 * @param pwm_timer LEDC timer to be used
 * @return motor_handler_t* Pointer to initialized motor handler, or NULL if allocation fails
 */
motor_handler_t *motor_new_handle(gpio_num_t dir_gpio, gpio_num_t pwm_gpio,
                                  ledc_channel_t pwm_channel, ledc_timer_t pwm_timer)
{
    motor_handler_t *handle = malloc(sizeof(motor_handler_t));
    if (!handle) return NULL;

    handle->dir_pin = dir_gpio;
    handle->pwm_pin = pwm_gpio;
    handle->pwm_channel = pwm_channel;

    // Configure direction GPIO as output and set to default forward direction
    gpio_set_direction(dir_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(dir_gpio, 0); // Default: forward
    gpio_set_direction(GPIO_NUM_18, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_18, 0); // Default: disable

    // Configure PWM timer: 12-bit resolution, 10 kHz frequency
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = pwm_timer,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .freq_hz = 10000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_config);

    // Configure PWM channel with initial duty = 0
    ledc_channel_config_t channel_config = {
        .gpio_num = pwm_gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = pwm_channel,
        .timer_sel = pwm_timer,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&channel_config);

    return handle;
}

/**
 * @brief Free memory allocated for the motor handler
 * 
 * @param self Pointer to the motor handler to delete
 */
void motor_delete_handle(motor_handler_t *self) {
    if (self) free(self);
}

/**
 * @brief Enable or disable the motor
 * 
 * @param self Pointer to motor handler
 * @param enable true to enable (set 50% duty), false to disable (0% duty)
 */
void motor_set_enable(motor_handler_t *self, bool enable) {
    gpio_set_level(GPIO_NUM_18, enable);
}

/**
 * @brief Set motor direction
 * 
 * @param self Pointer to motor handler
 * @param forward true for forward, false for reverse
 */
void motor_set_direction(motor_handler_t *self, bool forward) {
    gpio_set_level(self->dir_pin, forward ? 0 : 1);
}

/**
 * @brief Set motor speed using 12-bit duty cycle
 * 
 * @param self Pointer to motor handler
 * @param duty_cycle Duty cycle value (0–4095 for 12-bit resolution)
 */
void motor_set_speed(motor_handler_t *self, uint16_t duty_cycle) {
    if (duty_cycle > 4095) duty_cycle = 4095;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, self->pwm_channel, duty_cycle);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, self->pwm_channel);
}

/**
 * @brief Hàm điều khiển động cơ
 * 
 * @param motor Con trỏ đến động cơ
 * @param speed Tốc độ động cơ
 */
void motor_control(motor_handler_t *motor, int speed) {
    if (speed >= 3000) speed = 3000;
    if (speed <= -3000) speed = -3000;
    if (speed >= 0) {
        motor_set_direction(motor, 1); // Chạy tới
        motor_set_speed(motor,  4095 - speed); // Tốc độ
    } else {
        motor_set_direction(motor, 0); // Chạy lùi
        motor_set_speed(motor, 4095 + speed); // Tốc độ
    }
}