
#include <iostream>
using namespace std;

class motor {
    protected:
    float a,k,b,w,j,V;
    float theta;
    float Vconst;

    public:
    motor(){}
    motor(float k,float b, float J, float Vconst){
        this->k = k;
        this->b = b;
        this->j = J;
        this->V = 0;
        this->Vconst = Vconst;
        //Initially thereno rotation 
        this->a = 0;
        this->w = 0;
        this->theta = 0;
    }
    float get_omega()
    {
        return this->w;
    }

    void set_Voltage(float V){
        this->V = V;
    }

    void reflect_motor_rotation(float dt){
            a = (k*V - b*w) / j;
            w = w + a * dt;
            theta = theta + w * dt;
    }

};





class PIDtop
{
    protected:
    unsigned short kP,kI,kD;        //values from 0 to 65535
    float xt;           //xt = desired angle/output, yt = present angle/output
    float et;    //error signal
    
    public:
    //default constructor
    PIDtop(){}

    PIDtop(unsigned short kP, unsigned short kI, unsigned short kD){
        this->kD = kD;
        this->kI = kI;
        this->kP = kP;
    }

    virtual void ComputeOut(float dt){
        cout<<"Base class Compute"<<endl;
    }

    void get_refrence_signal(float xt){
        this->xt = xt;
    }
    void set_refrence_angle(float xt){
        this->xt = xt;
    }
};

class ProportionalControl : public PIDtop, public motor{
    public:
    ProportionalControl(){}
    ProportionalControl(unsigned short KP,float k, float b, float j,float Vconst) : PIDtop(KP,0,0), motor(k,b,j,Vconst){
    }

    
   
    void update_error_signal(){
        this->et = this->xt - this->theta;
    }
    void display(){
        update_error_signal();
        cout<<"[P] V:"<<V<<"\t\tTheta= "<<theta<<"\t\t x: "<<xt<<"\t\t e: "<<et<<endl;
        cout<<"[P] w: "<<w<<"\t\ta: "<<a<<endl;
    }


    //P CONTROLLER
    void Controller(){
        
        update_error_signal();
        if(et == 0 || et <0.05)
            V = 0;
        else 
            V = kP * Vconst;
    }
};

int main(){
    //motor parameters:- 
    float a,k,b,w,j,V,theta;

    //PID parameters:-
    unsigned short kP,kI,kD; 
    float xt,yt; 
    float et = 0;

    //Other values
    float Vconst;
    float dt =0.25;//in seconds

    //INPUT
    // cout<<"Enter motor constant(k), Friction Coefficiente(b) and moment of inertia (J)"<<endl;
    // cin>>k>>b>>j;

    cout<<"Enter Kp,ki,kd: "<<endl;
    // cin>>kP>>kI>>kD;
    cin>>kP;

    cout<<"Enter Desired Output Angle:- "<<endl;
    cin>>xt;

    cout<<"Enter the available voltage value:- "<<endl;
    cin>>Vconst;

    
    ProportionalControl p1(kP,2,3,3,Vconst);
    int counter = 0;
    p1.set_refrence_angle(xt);
  
  
    while(counter <= 200) //avoid infinte loop condition 
    {
        
        p1.display();
        p1.Controller();
        p1.reflect_motor_rotation(dt);
        if(p1.get_omega() <= 0.0005)
            break;
        counter ++;
    }
    





    return 0;
}