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
    void setNewPose_xy( Point2D );
    void setNewPose( POSE );
    void setVelocity( );

    // GETS


    // OPERATIONAL CONTROL FUNCTION
    void controlLoopFunc();
    void updateLoop();
    void updateDt();
    void navUpdate();
    void sensorUpdate();
    void missionUpdate();
    void updateCurrentTask();
    void sendMotorCommands();
    void publishPoseMessages();


    // UTILS
    double dist_to_point( Point2D );
    double dist_to_pose( POSE );





};


#endif // BAEBOTMASTER_H_INCLUDED
