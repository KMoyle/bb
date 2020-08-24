#ifndef BAEBOTMASTER_H_INCLUDED
#define BAEBOTMASTER_H_INCLUDED

#include "baebot_global.h"
#include "ros/ros.h"
#include <math.h>


class BaeBotMaster {

    private:

    // Subs and pubs


    public:

    POSE        pose;
    POSE        poseDmd;


    // CTOR & DTOR
    BaeBotMaster( ros::NodeHandle* );
    ~BaeBotMaster();

    // SETS
    void setNewGoal( POSE );
    void setVelocity( POSE );


    // UTILS
    double dist2Point( POSE , POSE );





};


#endif // BAEBOTMASTER_H_INCLUDED
