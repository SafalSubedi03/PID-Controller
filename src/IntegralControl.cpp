#include "IntegralControl.h"
#include <algorithm>

IntegralControl::IntegralControl() {}

IntegralControl::IntegralControl(float kI, float Vmax, float xt, float dt)
    : PIDTop(0, kI, 0, Vmax, xt, dt)
{
    past_error_sum = 0;
    Imax = Vmax / kI;
}

void IntegralControl::control()
{
 {
        // Calculate tentative output
        float tentative_sum = past_error_sum + et * dt;
        float tentative_Vout = kI * tentative_sum;

        // Anti-windup logic:
        // Only accumulate if output is not saturated,
        // or if error drives output back toward normal range.
        if ((tentative_Vout < Vmax && tentative_Vout > -Vmax) ||
            (tentative_Vout >= Vmax && et < 0) ||
            (tentative_Vout <= -Vmax && et > 0))
        {
            past_error_sum = tentative_sum;
        }
        else
        {
            // Slowly discharge the integrator to avoid freeze
            past_error_sum *= 0.9f;
        }

        // Clamp integral sum
        if (past_error_sum > Imax)
            past_error_sum = Imax;
        else if (past_error_sum < -Imax)
            past_error_sum = -Imax;

        // Compute final output
        Vout = kI * past_error_sum;

        // Clamp output to actuator limits
        if (Vout > Vmax)
            Vout = Vmax;
        else if (Vout < -Vmax)
            Vout = -Vmax;
    }
}
