
#include <iostream>
#include<cmath>


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
    static float xt;         // xt = desired angle/output, yt = present angle/output
    static float et;         // error signal
    float Vout, Vmax;
    float curr_theta;
    float dt;

public:
    // default constructor
    PIDtop() {}

    PIDtop(float kP, float kI, float kD, float Vmax,float xtt, float dt = 0.00025)
    {
        this->kD = kD;
        this->kI = kI;
        this->kP = kP;
        this->Vmax = Vmax;
        this->Vout = initial_motor_volt;
        this->dt = dt;
        xt = xtt;
    }

   
    static void update_error_signal(float curr_theta)
    {

        et = xt - curr_theta;
    }

    float get_vout()
    {
        return this->Vout;
    }
    float set_et(){
        et = et;
    }
};

float PIDtop::xt;
float PIDtop::et;


class ProportionalControl : public PIDtop
{
protected:
public:
    ProportionalControl() {}
    ProportionalControl(float KP, float Vmax,float xt,float dt) : PIDtop(KP, 0, 0, Vmax,xt,dt)
    {
    }
    // P CONTROLLER
    void Controller()
    {
        Vout = kP * et;
        Vout = (Vout > Vmax) ? Vmax :(Vout < -Vmax) ? -Vmax : Vout; // limits the output voltage to never exceed Vmax
    }
};

class integralControl : public PIDtop
{
    protected:
    float past_error_sum;

public:
    integralControl() {}
    integralControl(float kI, float Vmax,float xt,float dt) : PIDtop(0, kI, 0,Vmax,xt,dt)
    {
        this->past_error_sum = 0;
    }
    void control()
    {
        past_error_sum = past_error_sum + et * dt;
        cout<<"[IC]"<<past_error_sum<<endl;
        Vout = kI * past_error_sum;
        Vout = (Vout > Vmax) ? Vmax :(Vout < -Vmax) ? -Vmax : Vout; // limits the output voltage to never exceed Vmax

    }

};




int main()
{
    // motor parameters:-
    float a, k = 0.05, b = 0.005, w, j = 0.003, V;

    // PID parameters:-
    float kP = 2, kI=2, kD;
    float xt, yt;

    // Other values
    float Vmax = 12;
    float dt = 0.025; // in seconds
    float e = 0.1; //nearest to 1 position 

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
    NextSIM:
    cout<<"Enter the corresponding Number:- \n1.P Contorl.\n2.PI Contorl."<<endl;
    cin>>option;

    
    int counter = 0;
    switch (option)
    {
    case 1:
        {
            ProportionalControl p1(kP, Vmax,xt,dt);
            motor m1(k, b, j);
           
            while (counter <= 500) // avoid infinte loop condition
            {
                m1.display();
                ProportionalControl::update_error_signal(m1.get_theta());
                p1.Controller();
                m1.set_vin(p1.get_vout()); // Connect the motor input voltage supply with the output form the controller
                m1.reflect_motor_rotation(dt);
                if(fabs(m1.get_theta() - xt) < e){
                    cout<<"Reached xt at: "<<counter<<" iteration"<<endl;
                    break;
                }
                counter++;
            }
            break;
        }

    case 2: {
        //objects
        integralControl i1(kI,Vmax,xt,dt);
        ProportionalControl p1(kP, Vmax,xt,dt);
        motor m1(k, b, j);
        while (counter <= 500) // avoid infinte loop condition
            {
                m1.display();
                PIDtop::update_error_signal(m1.get_theta());
                p1.Controller();
                i1.control();

                //Get combined voltage value form the controller
                V = p1.get_vout()+i1.get_vout();
                V = (V > Vmax) ? Vmax :(V < -Vmax) ? -Vmax : V;

                // Connect the motor input voltage supply with the output form the controller
                m1.set_vin(V); 
                
                //Roate motor
                m1.reflect_motor_rotation(dt);

                //stop the itaration if necessary conditions are met.
                if(fabs(m1.get_theta() - xt) < e){
                    cout<<"Reached xt at: "<<counter<<" iteration"<<endl;
                    break;
                }
                counter++;
            }
            cout<<"Press any key to go back to menu"<<endl;
            cin>>option;
            if(option)
                goto NextSIM;
            break;
     
    }

    
    
    default:
        break;
    }
 
    return 0;
}