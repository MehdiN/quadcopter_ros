/* 

Main control loop for drone control

*/

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose.h"
#include "controller/PID.h"


void poseCallback(const geometry_msgs::Pose::ConstPtr& pose)
{
    float altitude = pose->position.z;
}


int main(int argc, char **argv)
{

    ros::init(argc,argv,"pub_control");
    ros::NodeHandle node;

    ros::Publisher control_pub = node.advertise<std_msgs::Float32>("cmd_z",1);
    ros::Subscriber control_pub = node.subscribe("/quadcopter/pose",1,poseCallback);

    ros::Rate rate(1);

    // Altitucde Controller 
    PID::PID pid;
    pid.init_or_reset_PID();
    pid.load_pid_params();
    
    double z_target = -10; // In NED configuration
    std_msgs::Float32 command;


    while( ros::ok())
    {
        

        // Simple PID for altitude control
        pid.set_target(z_target);
        command.data = pid.get_pid();
        
        control_pub.publish(command);
        ros::spinOnce();
        rate.sleep();
    }






}

