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
    _target_alt(0),
    _command(0)
    {

    }

    // Initialisation of the controller
    void init_controller();
    // Run the control algorithm
    void run();
    // set the target;
    void set_target(double target);
    // set the target;
    void set_altitude(double altitude);
    // return the command calculated by the controller
    double get_cmd(){return _command;};

    private:

    // PID
    PID pid;
    
    // controller state
    double _altitude;
    double _target_alt;
    double _command;



};

#endif
