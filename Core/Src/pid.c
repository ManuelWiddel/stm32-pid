#include "pid.h"



static float PID_Clamp(float value, float min, float max)
{
    if (value > max)
    {
        return max;
    }
  
    if (value < min)
    {
        return min;
    }
  
    return value;
}


void PID_Init(PID_Controller_t *pid,
              PID_Config_t *config)
{
    if ((pid == NULL) || (config == NULL))
    {
        return;
    }
  
    pid->config = config;

    /*
     * Basic configuration validation.
     */
    if (pid->config->Ts <= 0.0f)
    {
        pid->config->Ts = 0.001f;
    }
  
    if (pid->config->output_min > pid->config->output_max)
    {
        float temp = pid->config->output_min;
        pid->config->output_min = pid->config->output_max;
        pid->config->output_max = temp;
    }

    pid->integral = 0.0f;
    pid->previous_measurement = 0.0f;
    pid->output = 0.0f;
    pid->initialized = 0U;
}


void PID_Reset(PID_Controller_t *pid,
               float measurement)
{
    if (pid == NULL)
    {
        return;
    }
  
    pid->integral = 0.0f;
    pid->previous_measurement = measurement;
    pid->output = 0.0f;

    /*
     * Initialize derivative history.
     */
    pid->initialized = 1U;
}


void PID_SetSetpoint(PID_Controller_t *pid,
                     float setpoint)
{
    if ((pid == NULL) || (pid->config == NULL))
    {
        return;
    }
  
    pid->config->Setpoint = setpoint;
}


void PID_SetOutputLimits(PID_Controller_t *pid,
                         float min,
                         float max)
{
    if (pid == NULL)
    {
        return;
    }
  
    if (min > max)
    {
        float temp = min;
        min = max;
        max = temp;
    }

    pid->config->output_min = min;
    pid->config->output_max = max;

    /*
     * Make sure an existing integral contribution remains
     * inside the new output range.
     */
    pid->integral = PID_Clamp(
        pid->integral,
        min,
        max
    );
}


float PID_Update(PID_Controller_t *pid,
                 float measurement)
{
    float error;
    float proportional;
    float derivative;

    float integral_candidate;
    float output_candidate;

    float output;

    if (pid == NULL)
    {
        return 0.0f;
    }
  
    /*
     * On the first update, initialize the derivative history.
     */
    if (!pid->initialized)
    {
        pid->previous_measurement = measurement;
        pid->initialized = 1U;
    }

    /*
     * Error = setpoint - measurement.
     */
    error = pid->config->Setpoint - measurement;


    /*
     * ---------------------------------------------------------
     * Proportional term
     * ---------------------------------------------------------
     */
    proportional = pid->config->Kp * error;


    /*
     * ---------------------------------------------------------
     * Derivative term
     *
     * Derivative is calculated from the measurement rather than
     * the error. This prevents derivative kick when the setpoint
     * changes.
     * ---------------------------------------------------------
     */
    derivative =
        -pid->config->Kd *
        (measurement - pid->previous_measurement) /
        pid->config->Ts;


    /*
     * ---------------------------------------------------------
     * Calculate a candidate integral.
     * ---------------------------------------------------------
     */
    integral_candidate =
        pid->integral +
        pid->config->Ki * error * pid->config->Ts;


    /*
     * ---------------------------------------------------------
     * Calculate the output using the candidate integral.
     * ---------------------------------------------------------
     */
    output_candidate =
        proportional +
        integral_candidate +
        derivative;


    /*
     * ---------------------------------------------------------
     * Integral clamping anti-windup.
     *
     * If the output is saturated, only accept the integral
     * update if it is NOT pushing the output further into
     * saturation.
     *
     * Upper saturation:
     *
     *     output > max
     *     error > 0  -> don't integrate
     *
     * Lower saturation:
     *
     *     output < min
     *     error < 0  -> don't integrate
     *
     * If the error is driving the output back toward the valid
     * range, integration is allowed.
     * ---------------------------------------------------------
     */

    if (output_candidate > pid->config->output_max)
    {
        if (error < 0.0f)
        {
            /*
             * Controller is saturated high, but the error is
             * pulling the output downward.
             *
             * Allow integration so the integral can unwind.
             */
            pid->integral = integral_candidate;
        }
        /*
         * Otherwise don't update integral.
         */
    }
    else if (output_candidate < pid->config->output_min)
    {
        if (error > 0.0f)
        {
            /*
             * Controller is saturated low, but the error is
             * pulling the output upward.
             *
             * Allow integration so the integral can unwind.
             */
            pid->integral = integral_candidate;
        }
        /*
         * Otherwise don't update integral.
         */
    }
    else
    {
        /*
         * Output is inside the valid range.
         *
         * Normal integration.
         */
        pid->integral = integral_candidate;
    }


    /*
     * ---------------------------------------------------------
     * Calculate final output using the accepted integral.
     * ---------------------------------------------------------
     */
    output =
        proportional +
        pid->integral +
        derivative;


    /*
     * Output saturation.
     */
    output = PID_Clamp(
        output,
        pid->config->output_min,
        pid->config->output_max
    );


    /*
     * Save state.
     */
    pid->previous_measurement = measurement;
    pid->output = output;

    return output;
}


float PID_GetOutput(const PID_Controller_t *pid)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
  
    return pid->output;
}


float PID_GetIntegral(const PID_Controller_t *pid)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
  
    return pid->integral;
}

