#include "ProportionalControl.h"
#include <algorithm>

ProportionalControl::ProportionalControl() {}

ProportionalControl::ProportionalControl(float kP, float Vmax, float xt, float dt)
    : PIDTop(kP, 0, 0, Vmax, xt, dt) {}

void ProportionalControl::Controller()
{
   
        Vout = kP * et;
        Vout = (Vout > Vmax) ? Vmax : (Vout < -Vmax) ? -Vmax
                                                     : Vout; // limits the output voltage to never exceed Vmax
    

}
