#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pid_controller pid_controller_t;

/**
 * @brief Create a new PID controller
 * 
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param dt Sampling time in seconds
 * @return pid_controller_t* Pointer to the new PID controller
 */
pid_controller_t *pid_new_handle(float kp, float ki, float kd, float dt);

/**
 * @brief Delete the PID controller
 * 
 * @param self Pointer to the PID controller
 */
void pid_delete_handle(pid_controller_t *self);

/**
 * @brief Reset the PID internal state
 * 
 * @param self Pointer to the PID controller
 */
void pid_reset(pid_controller_t *self);

/**
 * @brief Set PID tuning parameters
 * 
 * @param self Pointer to the PID controller
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
void pid_set_gains(pid_controller_t *self, float kp, float ki, float kd);

/**
 * @brief Compute the PID output with anti-windup
 * 
 * @param self Pointer to the PID controller
 * @param setpoint Desired value
 * @param measured Measured value
 * @return int16_t Output value clipped in range [-4095, 4095]
 */
int16_t pid_compute(pid_controller_t *self, float setpoint, float measured);

#ifdef __cplusplus
}
#endif
