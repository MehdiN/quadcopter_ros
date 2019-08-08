#ifndef __QUADCOPTER_H__
#define __QUADCOPTER_H__

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nodelet/nodelet.h"
#include "quadcopter/Thrust.h"
#include "geometry_msgs/Accel.h"
#include <eigen3/Eigen/Core>

/*  
Class Quacopter for running Equation Of Motion of a  plus-shaped UUV 
It was intended to be loadable as a plugin for others projects by using the nodelet supclass
*/



namespace quadcopter
{
    class Quadcopter: public nodelet::Nodelet
    {
        
        public:
        
        Quadcopter();

         ~Quadcopter();
        
        typedef Eigen::Matrix<double,4,3> Tmatrix;
        static constexpr double g0{9.81}; //(9.80665;
        struct StateVector
        {
            Eigen::Vector3d pose; // x,y,z
            Eigen::Vector3d velocity; // vx vy vz
            Eigen::Vector3d angular_vel;// p q r
            Eigen::Vector4d quaternion; // qw qx qy qz
        };


        private:
        
        // Actions to perform called within the init method
        virtual void onInit();

        // Node Stuff
        ros::NodeHandle private_nh; // private node handler from nodelet manager
        ros::NodeHandle nh_in; // Node Handler to subcribe to external nodes
        ros::NodeHandle nh_out; // Node Handler to publish to external nodes
        
        // we publish the 3 following topics: pose vel and accel
        // we subcribe to the 3 following topic: thrust pos and accel

        // Publisher
        ros::Publisher _vel_pub;
        ros::Publisher _accel_pub;
        ros::Publisher _pose_pub;

        // Subcribers
        ros::Subscriber _thrust_sub;
        ros::Subscriber _pose_sub;
        ros::Subscriber _update_sub;

        // callback methods
        void poseCallback(const geometry_msgs::Pose::ConstPtr& input);
        void thrustCallback(const quadcopter::Thrust::ConstPtr& input);
        void updateCallback(const geometry_msgs::Accel::ConstPtr& derivative_input);


        // Dynamics and Kinematics methods
        void compute_eom();
        void update_transformation_matrix();

        // state vector
        // [x y z vx vy vz qw qx qy qz p q r]
        StateVector state;
        StateVector derivative;
        Tmatrix quat_matrix;
        Eigen::Matrix3d rotation_matrix;
        
        // Inertial coef
        double Ixx;
        double Iyy;
        double Izz;

        // rotors thrust
        Eigen::Vector4d thrust;

        // Parameters of the quadrotor
        double arm_len;
        double mass;
        double gamma;

        // Integration step
        double dt;

    };




}


#endif