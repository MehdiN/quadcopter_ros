// Simple PID controller with anti-windup
#include <iostream>
#include <cmath>
#include <algorithm>

class PID
{

public:

    PID();
    ~PID();

    float init_or_reset_PID();

    float get_pid();
    float get_proportional();
    float get_derivative();
    float get_integrator();
    void update_integrator_anti_wu();


private:

    float _kp;
    float _ki;
    float _kd;

    float _kt; // anti-windup gain 

    float _dt;

    float _derivative;
    float _integrator;
    float _target;
    float _error;
    float _output;
    float _sat_output;

    float low_val;
    float high_val;


};

