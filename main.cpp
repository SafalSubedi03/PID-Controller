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
    char goBack = 'b';
    int maxCounter = 500; //maximum while loop operation 
    int steadyvalues = 5; //number of responses to diosplay after reaching the target value
    int reachedval=0;
    
    cout << "Enter Desired Output Angle: ";
    cin >> xt;

    MAINMENU:
    cout << "Select Controller:\n1. P Control\n2. PI Control\n3. PID Control\n";
    cin >> option;

    Motor m1(k, b, j);
    int counter = 0;
    float V = 0;

    switch (option)
    {
    case 1:
    {
        cout<<"Enter Kp"<<endl;
        cin>>kP;
        ProportionalControl p1(kP, Vmax, xt, dt);
        bool first = true;
        while (counter <= maxCounter)
        {
            
            m1.display();
            PIDTop::update_error_signal(m1.get_theta());
            p1.Controller();
            m1.set_vin(p1.get_vout());
            m1.reflect_motor_rotation(dt);
            if (fabs(m1.get_theta() - xt) < e) {
                if(first){
                    reachedval = counter;
                    first = false;
                }

                counter = maxCounter-steadyvalues;
                steadyvalues--;
               
            }
            counter++;
        }
        if(first)
         reachedval = maxCounter;
        break;
  
    }
    case 2:
    {
        cout<<"Enter Kp ki"<<endl;
        cin>>kP>>kI;
        IntegralControl i1(kI, Vmax, xt, dt);
        ProportionalControl p1(kP, Vmax, xt, dt);
        bool first = true;
        while (counter <= maxCounter)
        {
            cout<<"["<<counter<<"]. ";
            m1.display();
            PIDTop::update_error_signal(m1.get_theta());
            p1.Controller();
            i1.control();
            V = p1.get_vout() + i1.get_vout();
            
            m1.set_vin(V);
            m1.reflect_motor_rotation(dt);
            if (fabs(m1.get_theta() - xt) < e) {
                if(first){
                    reachedval = counter;
                    first = false;
                }
                counter = maxCounter-steadyvalues;
                steadyvalues--;
            }
            counter++;
        }
         if(first)
        reachedval = maxCounter;
        break;
        
       
    }
    case 3:
    {
        cout<<"Enter Kp ki kd"<<endl;
        cin>>kP>>kI>>kD;
        IntegralControl i1(kI, Vmax, xt, dt);
        ProportionalControl p1(kP, Vmax, xt, dt);
        DerivativeControl d1(kD, Vmax, xt, dt);
        bool first = true;
        while (counter <= maxCounter)
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
            if (fabs(m1.get_theta() - xt) < e) {
                if(first){
                    reachedval = counter;
                    first = false;
                }
                counter = maxCounter-steadyvalues;
                steadyvalues--;
            }
            counter++;
        }
        if(first)
        reachedval = maxCounter;
        break;
       
    }
    default:
        cout << "Invalid Option!\n";
        break;
    }
        cout<<"Reahed the target value for first time at "<<reachedval<<endl;
        cout<<"Press q to quit programme"<<endl;
        cin>>goBack;
        if(goBack != 'q')
            goto MAINMENU;
    return 0;
}
