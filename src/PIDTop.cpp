#include "PIDTop.h"

float PIDTop::xt;
float PIDTop::et;

PIDTop::PIDTop() {}

PIDTop::PIDTop(float kP, float kI, float kD, float Vmax, float xtt, float dt)
{
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->Vmax = Vmax;
    this->Vout = 0.001;
    this->dt = dt;
    xt = xtt;
}

void PIDTop::update_error_signal(float curr_theta)
{
    et = xt - curr_theta;
}

float PIDTop::get_vout() const
{
    return Vout;
}
