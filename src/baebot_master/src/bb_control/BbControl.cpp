#include "BbControl.h"
#include "../BaeBotMaster.h"





BaeBotControl::BaeBotControl(){

        // INTEGRAL CONTROLLER GAIN
        Ki = 0.008;
        // PROPORTAIONAL CONTROLLER GAIN
        Kp = 1;
        // ANGLE DIFF GAIN
        Kn = 2;
        // PP ERROR
        pure_pursuit_error = 0;
        // bb intendend velocities



};

/**
 *
 * Controller force calculator
 *
 * @PARAM p  - pose structure
 * @PARAM pd - demanded pose structure
 * @PARAM bf - pointer to array of desired body forces
 *
 */


int   BaeBotControl::controllerPurePursuit( POSE p, POSE pd ){

    double d = 0.15; // pure pursuit stand back distance
    double theta_star;
    static double error_over_time = 0;
    double ang_diff;

    pure_pursuit_error = sqrt( pow( pd.x - p.x, 2) + pow( pd.y - p.y, 2) ) - d;

    error_over_time = error_over_time + pure_pursuit_error;

    // Proportional velocity
    p.v = Kp*pure_pursuit_error + Ki*error_over_time;

    //Proportional steering angle
    theta_star = atan2( ( pd.y - p.y ) , ( pd.x - p.x ) );

    ang_diff = angsDiff( theta_star, p.theta);

    p.w = Kn*ang_diff;

    //TODO -- Need a function to turn v and w into motor cmds, need to look at how they do it

    //TODO -- function to send motor cmds


};

std::pair<double, double>   BaeBotControl::controllerProportional( POSE p, POSE pd ){

    double ang_diff;
    double theta_star;
    std::pair<double, double> motor_cmds_vw;

    // Proportional control
    p.v = Kp*sqrt( pow( pd.x - p.x, 2) + pow( pd.y - p.y, 2) );

    //Proportional steering angle
    theta_star = atan2( ( pd.y - p.y ) , ( pd.x - p.x ) );

    ang_diff = angsDiff( theta_star, p.theta );

    p.w = Kn*ang_diff;


    motor_cmds_vw.first = p.v;
    motor_cmds_vw.second = p.w;

    return motor_cmds_vw;


};


double BaeBotControl::angsDiff( double theta_star, double theta_current){

    double ang_difference;

    if ( ( theta_star - theta_current ) > M_PI ){
        ang_difference = ( theta_star - theta_current ) - 2*M_PI;
    } else if ( (theta_star - theta_current ) < -M_PI ){
        ang_difference = ( theta_star - theta_current ) + 2*M_PI;
    } else {
        ang_difference = ( theta_star - theta_current );
    }
    return ang_difference;

};




























