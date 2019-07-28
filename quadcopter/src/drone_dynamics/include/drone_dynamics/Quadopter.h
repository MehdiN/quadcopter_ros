#ifndef __QUADCOPTER_H__
#define __QUADCOPTER_H__

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nodelet/nodelet.h"
#include "eigen3/Eigen/Core"


namespace quadcopter
{
    class Quadcopter: public nodelet::Nodelet
    {
        static constexpr double mass = 15.52;
        static constexpr double arm_len = 0.25;  

        public:
            virtual void onInit();




        private:

        Eigen::Vector3d linear_accel;
        Eigen::Vector3d angular_accel;
        Eigen::Vector3d angular_rate;
        Eigen::Vector3d velocity;
        
        Eigen::Vector4d thrust;



    };




}






#endif