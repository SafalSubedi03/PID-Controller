#include "Motor.h"

float initial_motor_volt = 0.001;

Motor::Motor() {}
Motor::Motor(float k, float b, float J)
{
    this->k = k;
    this->b = b;
    this->j = J;
    this->V = initial_motor_volt;
    this->a = 0;
    this->w = 0;
    this->theta = 0;
}

float Motor::get_theta() const { return theta; }
float Motor::get_omega() const { return w; }

void Motor::set_vin(float Vin) { V = Vin; }

void Motor::display() const
{
    cout << "[M] Vin:" << V << "\t\tTheta= " << theta << "\t\tw: " << w << endl;
}

void Motor::reflect_motor_rotation(float dt)
{
    a = (k * V - b * w) / j;
    w += a * dt;
    theta += w * dt;
}
