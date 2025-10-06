#ifndef MOTOR_H
#define MOTOR_H

#include <iostream>
using namespace std;

extern float initial_motor_volt;

class Motor
{
protected:
    float a, k, b, w, j, V;
    float theta;

public:
    Motor();
    Motor(float k, float b, float J);
    void set_vin(float Vin);
    float get_theta() const;
    float get_omega() const;
    void display() const;
    void reflect_motor_rotation(float dt);
};

#endif
