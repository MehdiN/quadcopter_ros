#ifndef _ALTITUDE_CONTROLLER_
#define _ALTITUDE_CONTROLLER_

#include "controller/PID.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose.h"


class AltitudeController
{


    public:

    AltitudeController():
    _altitude(0),
    _target(0),
    _command(0)
    {

    }

    // Initialisation of the controller
    void init();

    // Run the control algorithm
    void run();
    // set target;
    void set_target(float target){_target=target;};
    // get command
    double get_cmd(){return _command;};

    // callback for subcriber;
    void callback(const geometry_msgs::Pose::ConstPtr& pose);
    
    private:

    // PID
    PID pid;
    
    // controller state
    double _altitude;
    double _target;
    double _command;



};

#endif
