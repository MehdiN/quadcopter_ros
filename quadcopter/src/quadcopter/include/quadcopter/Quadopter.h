#ifndef __QUADCOPTER_H__
#define __QUADCOPTER_H__

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nodelet/nodelet.h"
#include "quadcopter/Thrust.h"
#include "quadcopter/Rate.h"
#include <eigen3/Eigen/Core>



namespace quadcopter
{
    class Quadcopter: public nodelet::Nodelet
    {
        
        public:
        
        Quadcopter():
        Ixx(0.1),
        Iyy(0.1),
        Izz(0.1),
        mass(0.1),
        gamma(0.1),
        arm_len(0.1)
        {
            state.pose.setZero();
            state.quaternion = Eigen::Vector4d(1.0,0,0,0);
            state.velocity.setZero();
            state.angular_vel.setZero();
            derivative.angular_vel.setZero();
            derivative.pose.setZero();
            derivative.velocity.setZero();
            derivative.quaternion.setZero();
            rotation_matrix.setIdentity();
            quat_matrix.setRandom();
        }
        
        typedef Eigen::Matrix<double,4,3> Tmatrix;
        static constexpr double g0 = 9.80665;
        struct StateVector
        {
            Eigen::Vector3d pose; // x,y,z
            Eigen::Vector3d velocity; // vx vy vz
            Eigen::Vector3d angular_vel;// p q r
            Eigen::Vector4d quaternion; // qw qx qy qz
        };


        private:
        
        // Actions to perform called withing the init method
        void onInit();

        // Node Stuff
        ros::NodeHandle private_nh; // private node handler from nodelet manager
        ros::NodeHandle nh_in; // Node Handler to subcribe to external nodes
        ros::NodeHandle nh_out; // Node Handler to publish to external nodes
        
        // Publisher
        ros::Publisher _vel_pub;
        ros::Publisher _accel_pub;
        ros::Publisher _pose_pub;

        // Subcribers
        ros::Subscriber _thrust_sub;
        ros::Subscriber _pose_sub;
        ros::Subscriber _update_sub;

        // callback methods
        void poseCallback(const geometry_msgs::PoseStampedConstPtr &input); // placeholder callback function
        void thrustCallback(const quadcopter::ThrustConstPtr& input);
        void updateCallback(const quadcopter::RateConstPtr& derivative_input);


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