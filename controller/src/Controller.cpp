#include "controller/Controller.h"
#include "controller/AltitudeController.h"



void Controller::initialize()
{
    _altitude_control->init_controller();
}

void Controller::update_pose(const geometry_msgs::PoseConstPtr pose)
{
    _altitude = pose->position.z;
    _altitude_control->set_altitude(_altitude);

}

void Controller::update_target()
{
    double tar_z = -1.0;
    _altitude_control->set_target(tar_z);
}

void Controller::run_controller()
{

    _altitude_control->run();
    double cmd_z(0);

    cmd_z = _altitude_control->get_cmd();

    _linear.x = 0;
    _linear.y = 0;
    _linear.z = cmd_z;
    _angular.x = 0;
    _angular.y = 0;
    _angular.z = 0;

    _linear_cmd.publish(_linear);
    _angular_cmd.publish(_angular);
}