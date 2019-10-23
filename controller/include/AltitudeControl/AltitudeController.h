#include "controller/PID.h"
#include "ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose.h"


class AltitudeController
{


    public:

    AltitudeController():
    command(0.0),
    target(0.0),
    altitude(0.0)
    {};


    ~AltitudeController();

    // Initialisation of the controller
    void init();
    void load_parameters();

    //publish
    void publish();
    void subcribe();

    // Run the control algorithm
    void run();
    // set target;
    void set_target(float target){this->target=target;};
    // get command
    double get_cmd(){return command;};


    // callback for subcriber;
    void callback(const geometry_msgs::Pose::ConstPtr& pose);
    
    

    private:

    // PID
    PID pid;
    
    // controller state
    double altitude;
    double target;
    double command;



};
