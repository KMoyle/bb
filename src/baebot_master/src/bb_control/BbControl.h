#ifndef BBCONTROL_H_INCLUDED
#define BBCONTROL_H_INCLUDED

#include "ros/ros.h"
#include "../baebot_global.h"

#include <math.h>

class BaeBotControl {

public:


    BaeBotControl();

    int controllerPurePursuit( POSE, POSE );
    int controllerProportional( POSE, POSE );
    double angsDiff( double, double );

    // Maximum allowable motor commands
    double  MOTOR_MAX_MAIN_PERCENTAGE = 20.0;

    // Sum of control axis integrator
    double pure_pursuit_error;

    // Control gains (set from configuration file)
    double  Kp;
    double  Ki;
    double  Kn;

    // Velocity and anguluar velocity
    double v;
    double w;


private:


};
#endif // BAEBOTMASTER_H_INCLUDED
