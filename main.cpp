#include "PIDCLASS.h"
#include <iostream>
#include <cmath>
using namespace std;

int main()
{
    float k = 0.05, b = 0.005, j = 0.0003;
    float kP = 2, kI = 50, kD = 0.3;
    float xt, Vmax = 12, dt = 0.025, e = 0.001;
    unsigned short option;

    cout << "Enter Desired Output Angle: ";
    cin >> xt;

    cout << "Select Controller:\n1. P Control\n2. PI Control\n3. PID Control\n";
    cin >> option;

    Motor m1(k, b, j);
    int counter = 0;
    float V = 0;

    switch (option)
    {
    case 1:
    {
        ProportionalControl p1(kP, Vmax, xt, dt);
        while (counter <= 500)
        {
            m1.display();
            PIDTop::update_error_signal(m1.get_theta());
            p1.Controller();
            m1.set_vin(p1.get_vout());
            m1.reflect_motor_rotation(dt);
            if (fabs(m1.get_theta() - xt) < e) break;
            counter++;
        }
        break;
    }
    case 2:
    {
        IntegralControl i1(kI, Vmax, xt, dt);
        ProportionalControl p1(kP, Vmax, xt, dt);
        while (counter <= 500)
        {
            m1.display();
            PIDTop::update_error_signal(m1.get_theta());
            p1.Controller();
            i1.control();
            V = p1.get_vout() + i1.get_vout();
            
            m1.set_vin(V);
            m1.reflect_motor_rotation(dt);
            if (fabs(m1.get_theta() - xt) < e) break;
            counter++;
        }
        break;
    }
    case 3:
    {
        IntegralControl i1(kI, Vmax, xt, dt);
        ProportionalControl p1(kP, Vmax, xt, dt);
        DerivativeControl d1(kD, Vmax, xt, dt);
        while (counter <= 500)
        {
            m1.display();
            PIDTop::update_error_signal(m1.get_theta());
            p1.Controller();
            i1.control();
            d1.control();
            V = p1.get_vout() + i1.get_vout() + d1.get_vout();
            V = (V > Vmax) ? Vmax : (V < -Vmax) ? -Vmax
                                                     : V; 
            m1.set_vin(V);
            m1.reflect_motor_rotation(dt);
            if (fabs(m1.get_theta() - xt) < e) break;
            counter++;
        }
        break;
    }
    default:
        cout << "Invalid Option!\n";
    }

    cout << "Reached target angle in " << counter << " iterations.\n";
    return 0;
}
