/* 

Main control loop for drone control

*/

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "controller/Controller.h"
#include "controller/AltitudeController.h"

int main(int argc, char **argv)
{

    ros::init(argc,argv,"controller");
    ros::NodeHandle node;
    AltitudeController altitude_controller;
    Controller main_controller(&node,&altitude_controller);

    // Altitude controller;
    main_controller.initialize();

    // initialize the subcriber and the publisher
    // ros::Publisher linear_cmd = node.advertise<std_msgs::Float32>("cmd_lin",1);
    // ros::Publisher angular_cmd = node.advertise<geometry_msgs::Vector3>("cmd_ang",1);
    // ros::Subscriber control_sub = node.subscribe("/quadcopter/pose",1,&AltitudeController::callback,&altitude_controller);
    ros::Subscriber main_control_sub = node.subscribe("/quadcopter/pose",1,&Controller::update_pose,&main_controller);

    
    //Timer to publish command every 1s;
    ros::Timer timerPublishCmd = node.createTimer(ros::Duration(1.0),std::bind(&Controller::run_controller, &main_controller));
    //Timer to read setpoints
    ros::Timer timerUpdateTarget = node.createTimer(ros::Duration(1.0),std::bind(&Controller::update_target,&main_controller));
    
    
    
    // ros::Rate rate(1);

    // double z_target = -1.5; // In NED configuration
    // geometry_msgs::Vector3 linear;
    // geometry_msgs::Vector3 angular;

    // while(ros::ok())
    // {
        
    //     // 
    //     altitude_controller.set_target(z_target);
    //     altitude_controller.run();
        
    //     // affect the linear command
    //     linear.x = 0;
    //     linear.y = 0;
    //     linear.z = altitude_controller.get_cmd();
        
    //     // affect the angular command
    //     angular.x = 0;
    //     angular.y = 0;
    //     angular.z = 0;

    //     linear_cmd.publish(linear);
    //     angular_cmd.publish(angular);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    ros::spin();




}

