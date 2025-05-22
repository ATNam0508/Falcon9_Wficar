#include "pid.h"
#include <stdlib.h>     // malloc, free
#include <string.h>     // memset
#include <math.h>       // fminf, fmaxf

#define PID_OUTPUT_MIN   (-4095)
#define PID_OUTPUT_MAX   (4095)

/**
 * @brief Internal structure for PID controller
 */
struct pid_controller {
    float kp;
    float ki;
    float kd;
    float dt;

    float integral;
    float prev_error;
};

/**
 * @brief Create a new PID controller
 */
pid_controller_t *pid_new_handle(float kp, float ki, float kd, float dt) {
    if (dt <= 0.0f) return NULL;

    pid_controller_t *self = (pid_controller_t *)malloc(sizeof(pid_controller_t));
    if (!self) return NULL;

    self->kp = kp;
    self->ki = ki;
    self->kd = kd;
    self->dt = dt;

    self->integral = 0.0f;
    self->prev_error = 0.0f;

    return self;
}

/**
 * @brief Delete the PID controller and free memory
 */
void pid_delete_handle(pid_controller_t *self) {
    if (self) {
        free(self);
    }
}

/**
 * @brief Reset internal integral and derivative state
 */
void pid_reset(pid_controller_t *self) {
    if (self) {
        self->integral = 0.0f;
        self->prev_error = 0.0f;
    }
}

/**
 * @brief Set PID gains (Kp, Ki, Kd)
 */
void pid_set_gains(pid_controller_t *self, float kp, float ki, float kd) {
    if (self) {
        self->kp = kp;
        self->ki = ki;
        self->kd = kd;
    }
}

/**
 * @brief Compute the PID output with anti-windup
 * 
 * @param self Pointer to PID controller
 * @param setpoint Desired value
 * @param measured Actual measured value
 * @return int16_t Output control value (clipped to -4095 ~ 4095)
 */
int16_t pid_compute(pid_controller_t *self, float setpoint, float measured) {
    if (!self) return 0;

    float error = setpoint - measured;

    // Proportional term
    float p_term = self->kp * error;

    // Tentative integral term (with anti-windup later)
    float i_term = self->integral + self->ki * error * self->dt;

    // Derivative term
    float derivative = (error - self->prev_error) / self->dt;
    float d_term = self->kd * derivative;

    // Compute raw output
    float output = p_term + i_term + d_term;

    // Saturate output and apply anti-windup
    if (output > PID_OUTPUT_MAX) {
        output = PID_OUTPUT_MAX;
    } else if (output < PID_OUTPUT_MIN) {
        output = PID_OUTPUT_MIN;
    } else {
        // Accept new integral only if not saturated
        self->integral = i_term;
    }

    // Update previous error
    self->prev_error = error;

    return (int16_t)(output);
}

/**
 * @brief Example of using the PID controller
 * @code
#include "pid_controller.h"

pid_controller_t *pid = NULL;

void app_main(void) {
    float kp = 1.2f;
    float ki = 0.8f;
    float kd = 0.05f;
    float dt = 0.01f;  // thời gian lấy mẫu 10ms

    pid = pid_new_handle(kp, ki, kd, dt);
    if (!pid) {
        printf("PID init failed\n");
        return;
    }
} 

float setpoint = 100.0f;    // giá trị mong muốn (ví dụ tốc độ)
float measured = 92.5f;     // giá trị đo được từ encoder hoặc sensor

int16_t output = pid_compute(pid, setpoint, measured);

// Đưa output ra động cơ (ví dụ PWM)
motor_set_speed(motor_handle, abs(output));
motor_set_direction(motor_handle, output >= 0);
 */