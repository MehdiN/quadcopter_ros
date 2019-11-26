#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose.h"
#include "controller/AltitudeController.h"


// Controller Interface for subcontrol

class Controller
{

public:

    Controller(ros::NodeHandle *nh, AltitudeController *altitude_control)
    {

        _linear.x = 0;
        _linear.y = 0;
        _linear.z = 0;
        _angular.x = 0;
        _angular.y = 0;
        _angular.z = 0;

        _linear_cmd = nh->advertise<geometry_msgs::Vector3>("/cmd_lin",1);
        _angular_cmd = nh->advertise<geometry_msgs::Vector3>("/cmd_ang",1);


    }

void initialize();
void update_pose(const geometry_msgs::PoseConstPtr pose);
void update_target();
void run_controller();

private:

    AltitudeController *_altitude_control;

    geometry_msgs::Vector3 _linear;
    geometry_msgs::Vector3 _angular;

    geometry_msgs::Pose _pose;

    ros::Publisher _linear_cmd;
    ros::Publisher _angular_cmd;

    // system state
    double _altitude;


};

#endif