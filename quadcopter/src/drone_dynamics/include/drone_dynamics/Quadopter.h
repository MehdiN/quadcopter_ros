#ifndef __QUADCOPTER_H__
#define __QUADCOPTER_H__

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nodelet/nodelet.h"
#include "nodelet_topic_tools/nodelet_throttle.h"
#include <eigen3/Eigen/Core>
#include <tf2_eigen>


namespace quadcopter
{
    class Quadcopter: public nodelet::Nodelet
    {
        static constexpr double mass = 2.3;
        static constexpr double arm_len = 0.25;  

        public:
            virtual void onInit();
        
        void update_dynamics()
        void compute_rotation_matrix();
        
        struct StateVector
        {
            Eigen::Vector3d pose // x,y,z
            Eigen::Vector3d velocity; // vx vy vz
            Eigen::Vector3d angular_vel;// p q r
            Eigen::Vector4d quaternion; // qw qx qy qz
        };
        


        private:

        // callback methods
        void callback(const geometry_msgs::QuaternionConstPtr& input);
        void callback2(const std_msgs::Float64Ptr input);


        // nodelet stuff
        ros::NodeHandle private_nh;
        ros::Subscriber _sub;
        ros::Publisher _pub;


        // state vector
        // [x y z vx vy vz qw qx qy qz p q r]
        StateVector state;
        StateVector derivative;
        Eigen::Matrix3d rotation_matrix;

        
        // Inertial coef
        double Ixx;
        double Iyy;
        double Izz;

        //
        Eigen::Vector4d thrust;




    };




}






#endif