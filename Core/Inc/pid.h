#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float Ts;
    float Setpoint;

    float output_min;
    float output_max;

} PID_Config_t;


typedef struct
{
    PID_Config_t *config;

    float integral;
    float previous_measurement;
    float output;

    uint8_t initialized;

} PID_Controller_t;



/**
 * @brief Initialize PID controller.
 */
void PID_Init(PID_Controller_t *pid,
              PID_Config_t *config);


/**
 * @brief Reset PID state.
 *
 * @param measurement Current process measurement.
 */
void PID_Reset(PID_Controller_t *pid,
               float measurement);


/**
 * @brief Set controller setpoint.
 */
void PID_SetSetpoint(PID_Controller_t *pid,
                     float setpoint);


/**
 * @brief Calculate one PID controller output.
 *
 * Must be called at the configured sample period.
 *
 * @param measurement Current process measurement.
 *
 * @return Saturated controller output.
 */
float PID_Update(PID_Controller_t *pid,
                 float measurement);


/**
 * @brief Change controller output limits.
 */
void PID_SetOutputLimits(PID_Controller_t *pid,
                         float min,
                         float max);


/**
 * @brief Get current controller output.
 */
float PID_GetOutput(const PID_Controller_t *pid);


/**
 * @brief Get current integral contribution.
 */
float PID_GetIntegral(const PID_Controller_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
