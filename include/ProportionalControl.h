#ifndef PROPORTIONALCONTROL_H
#define PROPORTIONALCONTROL_H

#include "PIDTop.h"

class ProportionalControl : public PIDTop
{
public:
    ProportionalControl();
    ProportionalControl(float kP, float Vmax, float xt, float dt);
    void Controller();
};

#endif
