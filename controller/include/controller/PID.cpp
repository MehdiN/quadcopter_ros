#include "controller/PID.h"



// Contructor

PID::PID():
_dt(0.01)
{
    _gains.kp = 1;
    _gains.ki = 0;
    _gains.kd = 0;
    _gains.kt = 0;
    _gains.tf = 1;

    _init = false;
}


void PID::init_or_reset_PID()
{
    _output = 0;
    _integrator = 0;
    _derivative = 0;
    _init = true;
}

void PID::set_target(double target)
{
    _target = target;
    _error = _target - _measure;
}

void PID::load_pid_params()
{
    double freq;
    ros::param::get("altitude_ctl_values/kp",_gains.kp);
    ros::param::get("altitude_ctl_values/ki",_gains.ki);
    ros::param::get("altitude_ctl_values/kd",_gains.kd);
    ros::param::get("altitude_ctl_values/kt",_gains.kt);
    ros::param::get("altitude_ctl_values/lp_freq",freq);
    set_freq_filter(freq);

}


void PID::set_freq_filter(double freq)
{
    if(freq > 0)
    {
        _gains.tf = 1/freq;
    }
}

double PID::get_proportional()
{
    return _gains.kp * _error;
}

double PID::get_integrator()
{
    return _integrator;
}

void PID::update_integrator_anti_wu()
{
    // saturate output
    _sat_output = std::min(low_val,_output);
    _sat_output = std::max(high_val,_output);
    _integrator += _gains.ki * _dt * _error + _dt * _gains.kt * (_output - _sat_output);
}

double PID::get_derivative()
{
    _derivative = _gains.tf*_derivative - _gains.kd*(_measure-_old_measure);   
    _derivative /= (_gains.tf + _dt);
    return _derivative; 
}

double PID::get_pid()
{
    _output = get_proportional() + get_derivative() + _integrator;
    update_integrator_anti_wu();
    _old_measure = _measure;
    return _output;
}