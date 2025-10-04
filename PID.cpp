
#include <iostream>

using namespace std;
float initial_motor_volt = 0.001;
class motor
{
protected:
    float a, k, b, w, j, V;
    float theta;

public:
    motor() {}
    motor(float k, float b, float J)
    {
        this->k = k;
        this->b = b;
        this->j = J;
        this->V = initial_motor_volt;

        // Initially thereno rotation
        this->a = 0;
        this->w = 0;
        this->theta = 0;
    }
    float get_omega()
    {
        return this->w;
    }
    void set_vin(float Vin)
    {
        V = Vin;
    }
    float get_theta()
    {
        return theta;
    }
    void display()
    {

        cout << "[M] Vin:" << V << "\t\tTheta= " << theta << "\t\tw: " << w << endl;
    }

    void reflect_motor_rotation(float dt)
    {
        a = (k * V - b * w) / j;

        w = w + a * dt;
        theta = theta + w * dt;
    }
};

class PIDtop
{
protected:
    float kP, kI, kD; // values from 0 to 65535
    float xt;         // xt = desired angle/output, yt = present angle/output
    float et;         // error signal
    float Vout, Vmax;
    float curr_theta;
    float dt;

public:
    // default constructor
    PIDtop() {}

    PIDtop(float kP, float kI, float kD, float Vmax, float dt = 0.00025)
    {
        this->kD = kD;
        this->kI = kI;
        this->kP = kP;
        this->Vmax = Vmax;
        this->Vout = initial_motor_volt;
        this->dt = dt;
    }

    void set_refrence_angle(float xt)
    {
        this->xt = xt;
    }

    void update_error_signal(float curr_theta)
    {

        this->et = this->xt - curr_theta;
    }

    float get_vout()
    {
        return this->Vout;
    }
};

class ProportionalControl : public PIDtop
{
protected:
public:
    ProportionalControl() {}
    ProportionalControl(float KP, float Vmax) : PIDtop(KP, 0, 0, Vmax)
    {
    }
    // P CONTROLLER
    void Controller()
    {

        Vout = kP * et;

        Vout = (Vout > Vmax) ? Vmax : (Vout < -Vmax) ? -Vmax
                                                     : Vout; // limits the output voltage to never exceed Vmax
    }
};

class integralControl : public PIDtop
{
protected:
    float past_error_sum;

public:
    integralControl() {}
    integralControl(float kI, float Vmax) : PIDtop(0, kI, 0, Vmax)
    {
        this->past_error_sum = 0;
    }
    void control()
    {
        past_error_sum = past_error_sum + et * dt;
        Vout = kI * past_error_sum;
    }
};




int main()
{
    // motor parameters:-
    float a, k = 0.05, b = 0.005, w, j = 0.00008, V, theta;

    // PID parameters:-
    float kP = 2, kI, kD;
    float xt, yt;

    // Other values
    float Vmax = 12;
    float dt = 0.025; // in seconds

    // INPUT
    //  cout<<"Enter motor constant(k), Friction Coefficiente(b) and moment of inertia (J)"<<endl;
    //  cin>>k>>b>>j;

    // cout<<"Enter Kp,ki,kd: "<<endl;
    // // cin>>kP>>kI>>kD;
    // cin>>kP;

    cout << "Enter Desired Output Angle:- " << endl;
    cin >> xt;

    // cout<<"Enter the available voltage value:- "<<endl;
    // cin>>Vmax;

    //Case contorl 
    unsigned short option;
    cout<<"Enter the corresponding Number:- \n1.P Contorl.\n2.PI Contorl."<<endl;
    cin>>option;
    switch (option)
    {
    case 1:
        {
            ProportionalControl p1(kP, Vmax);
            motor m1(k, b, j);
            int counter = 0;
            p1.set_refrence_angle(xt);
            while (counter <= 50) // avoid infinte loop condition
            {
                m1.display();
                p1.update_error_signal(m1.get_theta());
                p1.Controller();
                m1.set_vin(p1.get_vout()); // COnnect the motor input voltage supply with the output form the controller
                m1.reflect_motor_rotation(dt);
                counter++;
            }
            break;
        }
    
    
    default:
        break;
    }
 
    return 0;
}