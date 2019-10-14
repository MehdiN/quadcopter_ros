// Simple PID controller with anti-windup
#include <iostream>
#include <cmath>
#include <algorithm>
#include "ros/ros.h"

class PID
{

public:

    PID();
    ~PID();

    struct Gain
    {
        double kp;
        double ki;
        double kd;
        double tf;
        double kt; // anti-windup gain 
    };
    

    void set_target(double target);
    void set_freq_filter(double freq);
    void load_pid_params();

    void init_or_reset_PID();
    double get_pid();
    double get_proportional();
    double get_derivative();
    double get_integrator();
    void update_integrator_anti_wu();


private:

    Gain _gains;

    double _dt;

    double _derivative;
    double _integrator;
    double _target;
    double _measure;
    double _old_measure;
    double _error;
    double _output;
    double _sat_output;
    double low_val;
    double high_val;

    // flags

    bool _init;


};

