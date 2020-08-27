#ifndef BAEBOTMASTER_H_INCLUDED
#define BAEBOTMASTER_H_INCLUDED

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud.h"
#include "laser_geometry/laser_geometry.h"


#include "baebot_global.h"
#include "bb_sensors/BbLaser.h"
#include <math.h>


class BaeBotMaster {

    private:

    // Subs
    ros::Subscriber laser_sub;
    ros::Subscriber image_sub;




    public:

    double sampleRate;
    ros::Duration dt;
    ros::Rate r = 10; //10Hz

    POSE        pose;
    POSE        poseDmd;
    POSE        poseGoal;
    Point2D     point_xy;


    //Pubs
    ros::Publisher rviz_pub;
    ros::Publisher im_alive_pub;




    // CTOR & DTOR
    BaeBotMaster( ros::NodeHandle* );
    ~BaeBotMaster();

    // SETS
    void setNewPose_xy( Point2D );
    void setNewPose( POSE );
    void setVelocity( );

    // GETS

    // CALLBACK FUNCTIONS
    void rpLidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    laser_geometry::LaserProjection projector;


    void cameraImageCallback(const sensor_msgs::Image::ConstPtr& img);


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
