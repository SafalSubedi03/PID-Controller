#ifndef PIDTOP_H
#define PIDTOP_H

class PIDTop
{
protected:
    float kP, kI, kD;
    static float xt;
    static float et;
    float Vout, Vmax;
    float dt;

public:
    PIDTop();
    PIDTop(float kP, float kI, float kD, float Vmax, float xtt, float dt = 0.00025);
    static void update_error_signal(float curr_theta);
    float get_vout() const;
};

#endif
