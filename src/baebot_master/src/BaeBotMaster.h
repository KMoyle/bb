#ifndef BAEBOTMASTER_H_INCLUDED
#define BAEBOTMASTER_H_INCLUDED

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/PointCloud.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "message_filters/subscriber.h"


#include "baebot_global.h"
#include "bb_sensors/BbLaser.h"
#include <math.h>


class BaeBotMaster {

    private:

    // Subs
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;
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
    ros::Publisher scan_pub_;

    //LASER FUNCTION
    laser_geometry::LaserProjection projector;
    tf::TransformListener listener;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;



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
