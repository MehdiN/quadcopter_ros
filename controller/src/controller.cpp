/* 

Main control loop for drone control

*/

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose.h"
#include "altitudeControl/AltitudeController.h"

int main(int argc, char **argv)
{

    ros::init(argc,argv,"pub_control");
    ros::NodeHandle node;

    AltitudeController altitude_controller;

    // Altitude controller;
    altitude_controller.init();

    // initialize the subcriber and the publisher
    ros::Publisher control_pub = node.advertise<std_msgs::Float32>("cmd_z",1);
    ros::Subscriber control_sub = node.subscribe("/quadcopter/pose",1,&AltitudeController::callback,&altitude_controller);

    ros::Rate rate(1);

    double z_target = -1.5; // In NED configuration
    std_msgs::Float32 command;


    while( ros::ok())
    {
        
        // 
        altitude_controller.set_target(z_target);
        altitude_controller.run();
        command.data = altitude_controller.get_cmd();
        
        control_pub.publish(command);
        ros::spinOnce();
        rate.sleep();
    }






}

