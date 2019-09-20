#include "controller/PID.h"



float PID::get_integrator()
{
    return _integrator;
}

void PID::update_integrator_anti_wu()
{
    // saturate output
    _sat_output = std::min(low_val,_output);
    _sat_output = std::max(high_val,_output);
    _integrator += _ki * _dt * _error + _dt * _kt * (_output - _sat_output);
}

// float PID::get_derivative()
// {
//     _derivative = (1 - (N*dt)/_td) * _derivative - K*N *(_target - last_target);
//     return _derivative; 
// }

float PID::get_pid()
{
    _output = get_proportional() + get_derivative() + _integrator;
    update_integrator_anti_wu();
    return _output;

    

}