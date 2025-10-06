#ifndef DERIVATIVECONTROL_H
#define DERIVATIVECONTROL_H

#include "PIDTop.h"

class DerivativeControl : public PIDTop
{
protected:
    float prev_et;

public:
    DerivativeControl();
    DerivativeControl(float kD, float Vmax, float xt, float dt);
    void control();
};

#endif
