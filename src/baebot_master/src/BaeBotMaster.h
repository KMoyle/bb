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
    POSE        poseGoal;
    Point2D     point_xy;



    // CTOR & DTOR
    BaeBotMaster( ros::NodeHandle* );
    ~BaeBotMaster();

    // SETS
    void set_new_pose_xy( Point2D );
    void set_new_pose( POSE );
    void set_velocity( );

    // GETS



    // UTILS
    double dist_to_point( Point2D );
    double dist_to_pose( POSE );





};


#endif // BAEBOTMASTER_H_INCLUDED
