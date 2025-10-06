#include "DerivativeControl.h"
#include <algorithm>

DerivativeControl::DerivativeControl() {}

DerivativeControl::DerivativeControl(float kD, float Vmax, float xt, float dt)
    : PIDTop(0, 0, kD, Vmax, xt, dt)
{
    prev_et = et;
}

void DerivativeControl::control()
{
    Vout = kD * ((et - prev_et) / dt);
     Vout = (Vout > Vmax) ? Vmax : (Vout < -Vmax) ? -Vmax
                                                     : Vout; 
    prev_et = et;
}
