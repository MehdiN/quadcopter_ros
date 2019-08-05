#include <pluginlib/class_list_macros.h>
#include "Quadopter.h"


namespace quadcopter
{
    void Quadcopter::onInit()
    {
        NODELET_DEBUG("initializing nodelet");
        // get the Node Handler
        private_nh = getPrivateNodeHandle();
        nh_in = ros::NodeHandle(getNodeHandle(),"quadcopter_in");
        nh_out = ros::NodeHandle(getNodeHandle(),"quadcopter_out");


        // Load parameters
        private_nh.getParam("Leg_len",arm_len);
        private_nh.getParam("Gamma",gamma);
        private_nh.getParam("Mass",mass);
        private_nh.getParam("Ixx",Ixx);
        private_nh.getParam("Iyy",Iyy);
        private_nh.getParam("Izz",Izz);


        // Init publisher and subcriber
        _thrust_sub = nh_in.subscribe("thrust",10,&thrustCallback,this);
        _pose_sub = private_nh.subscribe("pose",10,&poseCallback,this);
        _vel_pub = nh_out.advertise<quadcopter::RatePtr>("accel",1,true);
        _accel_pub = private_nh.advertise<quadcopter::RatePtr>("accel",1,true);
        _update_sub = private_nh.subscribe("accel",10,&updateCallback,this);
        _pose_pub = nh_out.advertise<geometry_msgs::PoseStampedPtr>("pose",1,true);

    }

    // callback methods

    void Quadcopter::thrustCallback(const quadcopter::ThrustConstPtr &input)
    {
        // update thrust from topic
        thrust(0) = input->t0;
        thrust(1) = input->t1;
        thrust(2) = input->t2;
        thrust(3) = input->t3;

        // compute updated eom
        compute_eom();

        // publish the data

        quadcopter::RatePtr derivative_vel;
        
        derivative_vel->u = derivative.velocity(0);
        derivative_vel->v = derivative.velocity(1);
        derivative_vel->w = derivative.velocity(2);
        derivative_vel->p = derivative.angular_vel(0);
        derivative_vel->q = derivative.angular_vel(1);
        derivative_vel->r = derivative.angular_vel(2);

        _accel_pub.publish(derivative_vel);


    }


    void Quadcopter::updateCallback(const quadcopter::RateConstPtr &derivative_input)
    {
        state.velocity(0) += dt * derivative_input->u;
        state.velocity(1) += dt * derivative_input->v;
        state.velocity(2) += dt * derivative_input->w;

        state.angular_vel(0) += dt * derivative_input->p;
        state.angular_vel(1) += dt * derivative_input->q;
        state.angular_vel(2) += dt * derivative_input->r;

        // TODO: publish updated velocity
        quadcopter::Rate velocity;
        
        velocity.u = state.velocity(0);
        velocity.v = state.velocity(1);
        velocity.w = state.velocity(2);
        velocity.p = state.angular_vel(0);
        velocity.q = state.angular_vel(1);
        velocity.r = state.angular_vel(2);

        _vel_pub.publish(velocity);

        // TODO: define transform matrix T
        Eigen::Matrix3Xd matrix_T;
        derivative.pose = rotation_matrix * state.velocity;
        derivative.quaternion = matrix_T * state.angular_vel;

        //update

        state.pose += dt*derivative.pose;
        state.quaternion += dt*derivative.quaternion;

        // normalize quaternion

        state.quaternion.normalize();

        // Update R & T matrices

        update_transformation_matrix();

        // publish updated POS

        geometry_msgs::PoseStampedPtr updated_pose(new geometry_msgs::PoseStamped);

        updated_pose->pose.position.x = state.pose(0);
        updated_pose->pose.position.y = state.pose(1);
        updated_pose->pose.position.z = state.pose(2);

        updated_pose->pose.orientation.w = state.quaternion(0);
        updated_pose->pose.orientation.x = state.quaternion(1);
        updated_pose->pose.orientation.y = state.quaternion(2);
        updated_pose->pose.orientation.z = state.quaternion(3);

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
        // Angular dynamic
        
        Eigen::Matrix3d A;
        Eigen::Matrix3Xd B(4);

        
        A << 0, state.angular_vel(2)*(Iyy-Izz)/Ixx ,0,
                0,  0, state.angular_vel(0)*(Izz-Ixx)/Iyy,
                state.angular_vel(1)*(Ixx-Iyy)/Izz, 0, 0;

        B << 0, -arm_len, 0, arm_len,
            arm_len, 0, -arm_len, 0,
            gamma,-gamma,gamma,-gamma;

        // dx = Ax+Bu

        // Compute angular acceleration
        derivative.angular_vel = A * state.angular_vel + B * thrust;

        Eigen::Vector3d g(0,0,g0);
        Eigen::Vector3d u(0,0,thrust.sum()/mass);

        // Linear dynamic
        
        // compute linear acceleration
        derivative.velocity = rotation_matrix.inverse() * g + u - state.angular_vel.cross(state.velocity);

    }


PLUGINLIB_EXPORT_CLASS(quadcopter::Quadcopter, nodelet::Nodelet)
}

