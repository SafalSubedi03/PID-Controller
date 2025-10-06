#ifndef INTEGRALCONTROL_H
#define INTEGRALCONTROL_H

#include "PIDTop.h"

class IntegralControl : public PIDTop
{
protected:
    float past_error_sum;
    float Imax;

public:
    IntegralControl();
    IntegralControl(float kI, float Vmax, float xt, float dt);
    void control();
};

#endif
