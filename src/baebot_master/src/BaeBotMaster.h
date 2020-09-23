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
#include <queue>


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

        // Points to check control and senors data
    std::queue<std::pair<double, double> > goto_points;
    //Motor cmds to publish
    std::pair<double, double> motor_cmds_vw;

    // USING DEBUG & SIM MODES
    bool SIM = false;
    bool DEBUG = true;

    //Initial movement bool
    bool we_are_off = false;
    //Threshold dist for Proportional control
    const double THRESH_DIST = 0.05; //5cm

    //Declaring state and control objects
    BaeBotControl baeBotControl;
    MISSION_MODE mission_status;
    SENSOR_STATUS sensor_status;

    //Sensors timeouts
    double sensorTimeOut = 5.0;
    double timeSinceLastLidarUpdate;
    double timeSinceLastCameraUpdate;
    double timeSinceLastPoseUpdate;

    // Timing infos
    double sampleRate = 30;
    ros::Duration dt;
    ros::Rate r = 30; //10Hz
    ros::Time* lastUpdateTime = NULL;

    //Declaring pose & posedmd infos
    POSE        pose;
    POSE        poseDmd;
    POSE        poseGoal;
    Point2D     point_xy;

    //Pubs
    ros::Publisher pose_pub;
    ros::Publisher motorDmd_pub;


    // CTOR & DTOR
    BaeBotMaster( ros::NodeHandle* );
    ~BaeBotMaster();

    // SETS
    void setNewPose_xy( Point2D );
    void setNewPose( POSE );

    // GETS

    // CALLBACK FUNCTIONS
    void rpLidarCallback(const sensor_msgs::LaserScan::ConstPtr& );
    void cameraImageCallback(const sensor_msgs::Image::ConstPtr& );
    void bbPoseCallback(const nav_msgs::Odometry::ConstPtr& );
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

    //DATA gathering functions to TEST ODOM
    void doASpin();
    void goStraight();

    // UTILS
    double dist_to_point( Point2D );
    double dist_to_pose( );





};


#endif // BAEBOTMASTER_H_INCLUDED
