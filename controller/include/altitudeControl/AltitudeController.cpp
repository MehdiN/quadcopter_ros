#include "AltitudeController.h"

void AltitudeController::init()
{
    // init the controller
    pid.init_or_reset_PID();
    // load the controller parameters
    pid.load_pid_params();

}

void AltitudeController::run()
{
    pid.set_target(_target);
    _command = pid.get_pid();
}

void AltitudeController::callback(const geometry_msgs::Pose::ConstPtr& pose)
{
    _altitude = pose->position.z;
}