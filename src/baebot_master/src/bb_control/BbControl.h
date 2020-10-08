#ifndef BBCONTROL_H_INCLUDED
#define BBCONTROL_H_INCLUDED

#include "ros/ros.h"
#include "../baebot_global.h"

#include <math.h>

class BaeBotControl {

public:


    BaeBotControl();
    // Different controllers
    std::pair<double, double> controllerPurePursuit( POSE, POSE );
    std::pair<double, double>  controllerProportional( POSE, POSE );
    double angsDiff( double, double );


    // Maximum allowable motor commands
    //double  MOTOR_MAX_MAIN_PERCENTAGE = 20.0;

    // Sum of control axis integrator
    double pure_pursuit_error;

    // Control gains (set from configuration file)
    double  Kp;
    double  Ki;
    double  Kn;

    // Velocity and anguluar velocity
    std::pair<double, double> motor_cmds_vw;
    double v;
    double w;

    // wrap around angle
    double ang_diff;
    double d = 0.01; // pure pursuit stand back distance (m)
    double theta_star;

private:

    const double MAX_FORWARD_VELOCITY = 0.06;
    const double MAX_ANGULAR_VELOCITY = 1.5;


};
#endif // BAEBOTMASTER_H_INCLUDED
