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

#include <actionlib/client/simple_action_client.h>
#include <baebot_path_planner/PathPlannerAction.h>

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

    baebot_path_planner::PathPlannerGoal pathgoal;
    baebot_path_planner::PathPlannerResultConstPtr pathResult;




    public:

        // Points to check control and senors data
    std::queue<std::pair<double, double> > goto_points;
    //Motor cmds to publish
    std::pair<double, double> motor_cmds_vw;

    // USING DEBUG & SIM MODES
    bool SIM{false};
    bool DEBUG{false};
    bool DEBUG_PLANNER{true};
    bool DEBUG_SM{false};
    bool DEBUG_POSE{false};


    //Initial movement bool
    bool we_are_off{false};
    //Threshold dist for Proportional control
    const double THRESH_DIST{0.05}; //5cm
    //Require new path from planner
    bool need_new_path{false};
    bool waiting_for_path{false};
    bool test_new_path{true};



    //Declaring state and control objects
    BaeBotControl baeBotControl;
    MISSION_MODE mission_status;
    SENSOR_STATUS sensor_status;

    //Sensors timeouts
    double sensorTimeOut{5.0};
    double timeSinceLastLidarUpdate;
    double timeSinceLastCameraUpdate;
    double timeSinceLastPoseUpdate;

    // Timing infos
    double sampleRate{30};
    ros::Duration dt;
    ros::Rate r{30}; //10Hz
    ros::Time* lastUpdateTime{nullptr};

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

    void setnewPathGoal( int x, int y ){
        pathgoal.goal_x;
        pathgoal.goal_y;
    }

    // GETS

    // CALLBACK FUNCTIONS
    void rpLidarCallback(const sensor_msgs::LaserScan::ConstPtr& );
    void cameraImageCallback(const sensor_msgs::Image::ConstPtr& );
    void bbPoseCallback(const nav_msgs::Odometry::ConstPtr& );
    void batteryStateCallback( const sensor_msgs::BatteryState::ConstPtr& );


    // OPERATIONAL CONTROL FUNCTION
    void controlLoopFunc();
    void pathPlannerLoopFunc();
    void updateLoop();
    void updateDt();
    void navUpdate();
    void sensorUpdate();
    void missionUpdate();
    void updateCurrentTask();
    void publishMotorCommands();
    void publishPoseMessages();


    //DATA gathering and TEST functions
    void doASpin();
    void goStraight();
    void pathPlannerTest();

    // UTILS
    double dist_to_point( Point2D );
    double dist_to_pose( );





};


#endif // BAEBOTMASTER_H_INCLUDED
