#ifndef _PID_H_
#define _PID_H_

// Simple PID controller with anti-windup
#include <iostream>
#include <cmath>
#include <algorithm>
#include "ros/ros.h"


class PID
{

public:

    PID();
    ~PID(){};

    struct Gain
    {
        double kp = 0;
        double ki = 0;
        double kd = 0;
        double tf = 0;
        double kt = 0; // anti-windup gain 
    };
    

    void compute_error(double target,double measure);
    void set_freq_filter(double freq);
    void set_saturator(double low,double high);
    bool load_pid_params();

    void init_or_reset_PID();
    double get_pid();
    double get_proportional();
    double get_derivative();
    double get_integrator();
    void update_integrator_anti_wu();

    void get_gains(Gain &gains);


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
    double _low_val;
    double _high_val;

    // flags

    bool _init;


};

#endif