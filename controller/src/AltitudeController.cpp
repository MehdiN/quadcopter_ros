#include "controller/AltitudeController.h"

void AltitudeController::init_controller()
{
    // init the controller
    std::cout << "INIT PID" << std::endl;
    pid.init_or_reset_PID();
    // load the controller parameters
    bool params_loaded = pid.load_pid_params();
    if(params_loaded)
    {
        std::cout << "SUCCESS" << std::endl;
    }
    else
    {
        std::cout << "FAILURE" << std::endl;
    }
    
    PID::Gain params;
    pid.get_gains(params);
    std::cout << "KP: " << params.kp << std::endl;
    std::cout << "KI: " << params.ki << std::endl;
    std::cout << "KD: " << params.kd << std::endl;

}

void AltitudeController::run()
{
    pid.compute_error(_target_alt,_altitude);
    _command = pid.get_pid();
}


void AltitudeController::set_target(double target)
{
    _target_alt = target;
}


void AltitudeController::set_altitude(double altitude)
{
   _altitude = altitude;
}