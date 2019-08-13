#include <pluginlib/class_list_macros.h>
#include "quadcopter/Quadcopter.h"

namespace quadcopter
{

    // Constructor
    Quadcopter::Quadcopter():
    Ixx(0.1),
    Iyy(0.1),
    Izz(0.1),
    mass(0.1),
    gamma(0.1),
    arm_len(0.1)
    {
        dt = 0.01;
        thrust.setZero();
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
    // destructor
    Quadcopter::~Quadcopter()
    {
    }

    // declare the  static constexpr outside class to avoid linkage error
    constexpr double Quadcopter::g0;

    void Quadcopter::onInit()
    {
        NODELET_DEBUG("initializing nodelet");
        
        // get the Node Handlers
        private_nh = getPrivateNodeHandle();
        // nh_in = ros::NodeHandle(getNodeHandle(),"quadcopter_in");
        // nh_out = ros::NodeHandle(getNodeHandle(),"quadcopter_out");
        public_nh = ros::NodeHandle(getNodeHandle(),"out");


        // Load parameters
        private_nh.getParam("Leg_len",arm_len);
        private_nh.getParam("Gamma",gamma);
        private_nh.getParam("Mass",mass);
        private_nh.getParam("Ixx",Ixx);
        private_nh.getParam("Iyy",Iyy);
        private_nh.getParam("Izz",Izz);


        const std::string nh_name = public_nh.getNamespace();


        // Init publisher and subcriber
    
        _vel_pub = public_nh.advertise<geometry_msgs::Accel>("vel",1,true);
        _accel_pub = public_nh.advertise<geometry_msgs::Accel>("accel",1,true);
        _pose_pub = public_nh.advertise<geometry_msgs::Pose>("pose",1,true);

        _thrust_sub = public_nh.subscribe("thrust",10,&quadcopter::Quadcopter::thrustCallback,this);
        _pose_sub = private_nh.subscribe(nh_name + "/pose",10,&quadcopter::Quadcopter::poseCallback,this);
        _update_sub = private_nh.subscribe( nh_name + "/accel",10,&quadcopter::Quadcopter::updateCallback,this);
        

    }

    // callback methods

    void Quadcopter::poseCallback(const geometry_msgs::Pose::ConstPtr &data)
    {
        
        NODELET_INFO("pose callback");
        
        state.pose(0) = data->position.x;
        state.pose(1) = data->position.y;
        state.pose(2) = data->position.z;


        state.quaternion(0) = data->orientation.w;
        state.quaternion(1) = data->orientation.x;
        state.quaternion(2) = data->orientation.y;
        state.quaternion(3) = data->orientation.z;
    }


    void Quadcopter::thrustCallback(const quadcopter::Thrust::ConstPtr &input)
    {

        NODELET_INFO("Get motor thrust callback");

        // update thrust from topic
        thrust(0) = input->t0;
        thrust(1) = input->t1;
        thrust(2) = input->t2;
        thrust(3) = input->t3;

        // compute updated eom
        compute_eom();

        // publish the data

        geometry_msgs::AccelPtr derivative_vel(new geometry_msgs::Accel);
        
        derivative_vel->linear.x = derivative.velocity(0);
        derivative_vel->linear.y = derivative.velocity(1);
        derivative_vel->linear.z = derivative.velocity(2);
        derivative_vel->angular.x = derivative.angular_vel(0);
        derivative_vel->angular.y = derivative.angular_vel(1);
        derivative_vel->angular.z = derivative.angular_vel(2);

        _accel_pub.publish(derivative_vel);


    }


    void Quadcopter::updateCallback(const geometry_msgs::Accel::ConstPtr &derivative_input)
    {

        NODELET_INFO("Integrate Vel");

        state.velocity(0) += dt * derivative_input->linear.x;
        state.velocity(1) += dt * derivative_input->linear.y;
        state.velocity(2) += dt * derivative_input->linear.z;

        state.angular_vel(0) += dt * derivative_input->angular.x;
        state.angular_vel(1) += dt * derivative_input->angular.y;
        state.angular_vel(2) += dt * derivative_input->angular.z;

        // TODO: publish updated velocity
        geometry_msgs::AccelPtr velocity(new geometry_msgs::Accel);
        
        velocity->linear.x = state.velocity(0);
        velocity->linear.y = state.velocity(1);
        velocity->linear.z = state.velocity(2);
        velocity->angular.x = state.angular_vel(0);
        velocity->angular.y = state.angular_vel(1);
        velocity->angular.z = state.angular_vel(2);

        _vel_pub.publish(velocity);

        
        derivative.pose = rotation_matrix * state.velocity;
        derivative.quaternion = quat_matrix * state.angular_vel;

        //update

        state.pose += dt*derivative.pose;
        state.quaternion += dt*derivative.quaternion;

        //Limit altitude Note: NED convention

        if(state.pose(2) < - MAX_ALT)
        {
            state.pose(2) = - MAX_ALT;
        } 
        else if(state.pose(2)>0)
        {
            state.pose(2) = 0;
        }

        // normalize Quaternion

        state.quaternion.normalize();

        // Update R & T matrices

        update_transformation_matrix();

        // publish updated POS

        geometry_msgs::PosePtr updated_pose(new geometry_msgs::Pose);

        updated_pose->position.x = state.pose(0);
        updated_pose->position.y = state.pose(1);
        updated_pose->position.z = state.pose(2);

        updated_pose->orientation.w = state.quaternion(0);
        updated_pose->orientation.x = state.quaternion(1);
        updated_pose->orientation.y = state.quaternion(2);
        updated_pose->orientation.z = state.quaternion(3);

        _pose_pub.publish(updated_pose);
    
    }

    void Quadcopter::update_transformation_matrix()
    {
        // update rotation matrix

        double q0,q1,q2,q3,sq1,sq2,sq3,q01,q02,q03,q12,q13,q23;

        q0 = state.quaternion(0);
        q1 = state.quaternion(1);
        q2 = state.quaternion(2);
        q3 = state.quaternion(3);
        
        
        sq1 = q1*q1;
        sq2 = q2*q2;
        sq3 = q2*q2;
        q01 = q0*q1;
        q02 = q0*q2;
        q03 = q0*q3;
        q12 = q1*q2;
        q13 = q1*q3;
        q23 = q2*q3;


        rotation_matrix(0,0) = 0.5-(sq2+sq3);
        rotation_matrix(0,1) = q12-q03;
        rotation_matrix(0,2) = q13+q02;
        rotation_matrix(1,0) = q12+q03;
        rotation_matrix(1,1) = 0.5-(sq1+sq3);
        rotation_matrix(1,2) = q23-q01;
        rotation_matrix(2,0) = q13-q02;
        rotation_matrix(2,1) = q23-q01;
        rotation_matrix(2,2) = 0.5-(sq1+sq2);

        rotation_matrix *= 2;

        quat_matrix(0,0) = -q1;
        quat_matrix(0,1) = -q2;
        quat_matrix(0,2) = -q3;
        quat_matrix(1,0) =  q0;
        quat_matrix(1,1) = -q3;
        quat_matrix(1,2) =  q2;
        quat_matrix(2,0) =  q3;
        quat_matrix(2,1) =  q0;
        quat_matrix(2,2) = -q1;
        quat_matrix(3,0) = -q2;
        quat_matrix(3,1) =  q1;
        quat_matrix(3,2) =  q0;

        quat_matrix *= 0.5;


    }


    void Quadcopter::compute_eom()
    {

        NODELET_INFO("Update EOM");

        // Angular dynamic
        
        Eigen::Matrix3d A;
        Eigen::Matrix<double,3,4> B;

        
        A << 0, state.angular_vel(2)*(Iyy-Izz)/Ixx ,0,
                0,  0, state.angular_vel(0)*(Izz-Ixx)/Iyy,
                state.angular_vel(1)*(Ixx-Iyy)/Izz, 0, 0;

        B << 0, -arm_len, 0, arm_len,
            arm_len, 0, -arm_len, 0,
            gamma,-gamma,gamma,-gamma;

        // dx = Ax+Bu

        // Compute angular acceleration
        derivative.angular_vel = A * state.angular_vel + B * thrust;

        Eigen::Vector3d g(0,0,9.81);
        Eigen::Vector3d u(0,0,thrust.sum()/mass);

        // Linear dynamic
        
        // compute linear acceleration
        derivative.velocity = rotation_matrix.inverse()*g + u - state.angular_vel.cross(state.velocity);

    }

PLUGINLIB_EXPORT_CLASS(quadcopter::Quadcopter, nodelet::Nodelet)
}

// PLUGINLIB_EXPORT_CLASS(quadcopter::Quadcopter, nodelet::Nodelet);