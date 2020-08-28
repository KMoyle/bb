#ifndef BBCONTROL_H_INCLUDED
#define BBCONTROL_H_INCLUDED

#include "ros/ros.h"
#include "../baebot_global.h"

class BaeBotControl {

public:


    BaeBotControl();

    int   controller( POSE, POSE, double*, int* );

    // Maximum allowable motor commands
    double  MOTOR_MAX_MAIN_PERCENTAGE = 20.0;

    // Sum of control axis integrator
    double  sumErrorVelX;
    double  sumErrorVelY;
    double  sumErrorYaw;

    // Control gains (set from configuration file)
    double  Kp;


private:


};
#endif // BAEBOTMASTER_H_INCLUDED
