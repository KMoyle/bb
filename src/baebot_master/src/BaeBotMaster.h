#ifndef BAEBOTMASTER_H_INCLUDED
#define BAEBOTMASTER_H_INCLUDED

#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/BatteryState.h"

#include "laser_geometry/laser_geometry.h"
#include "tf2_ros/transform_listener.h"

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include <image_transport/image_transport.h>

#include "baebot_global.h"
#include "bb_sensors/BbLaser.h"
#include "bb_control/BbControl.h"
#include <math.h>


class BaeBotMaster {

    private:

    // Subs
    ros::Subscriber laser_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber poseDmd_sub;
    ros::Subscriber battery_sub;
    image_transport::Subscriber image_sub;
    image_transport::ImageTransport it_;




    public:

    std::vector<std::pair<double, double> > goto_points;
    ros::Time goto_time;

    bool DEBUG = true;
    bool we_are_off = false;
    const double THRESH_DIST = 0.05; //5cm


    BaeBotControl baeBotControl;
    MISSION_MODE mission_status;
    SENSOR_STATUS sensor_status;


    //Sensors timeouts
    double sensorTimeOut;
    double timeSinceLastLidarUpdate;
    double timeSinceLastCameraUpdate;
    double timeSinceLastPoseUpdate;

    std::pair<double, double> motor_cmds_vw;

    double sampleRate = 30;
    ros::Duration dt;
    ros::Rate r = 30; //10Hz
    ros::Time* lastUpdateTime = NULL;

    POSE        pose;
    POSE        poseDmd;
    POSE        poseGoal;
    Point2D     point_xy;


    //Pubs
    ros::Publisher rviz_pub;
    ros::Publisher pose_pub;
    ros::Publisher poseDmd_pub;
    ros::Publisher motorDmd_pub;


    // CTOR & DTOR
    BaeBotMaster( ros::NodeHandle* );
    ~BaeBotMaster();

    // SETS
    void setNewPose_xy( Point2D );
    void setNewPose( POSE );
    void setVelocity( );

    // GETS

    // CALLBACK FUNCTIONS
    void rpLidarCallback(const sensor_msgs::LaserScan::ConstPtr& );
    //void rpLidarCallback(const sensor_msgs::MultiEchoLaserScan::ConstPtr& );
    void cameraImageCallback(const sensor_msgs::Image::ConstPtr& );
    void bbPoseCallback(const nav_msgs::Odometry::ConstPtr& );
    void bbPoseDmdCallback(const nav_msgs::Odometry::ConstPtr& );
    void batteryStateCallback( const sensor_msgs::BatteryState::ConstPtr& );


    // OPERATIONAL CONTROL FUNCTION
    void controlLoopFunc();
    void updateLoop();
    void updateDt();
    void navUpdate();
    void sensorUpdate();
    void missionUpdate();
    void updateCurrentTask();
    void publishMotorCommands();
    void publishPoseMessages();


    // UTILS
    double dist_to_point( Point2D );
    double dist_to_pose( );





};


#endif // BAEBOTMASTER_H_INCLUDED
