#include "BaeBotMaster.h"
#include "baebot_global.h"



BaeBotMaster::BaeBotMaster(ros::NodeHandle *nh ){


        //laser_sub = nh->subscribe<sensor_msgs::LaserScan>("/rplidar_scan" , 1, &BaeBotMaster::rpLidarCallback, this);
        laser_sub = nh->subscribe<sensor_msgs::LaserScan>("/scan" , 1, &BaeBotMaster::rpLidarCallback, this);


        ROS_WARN("ctor");
};

BaeBotMaster::~BaeBotMaster(){





};

void BaeBotMaster::controlLoopFunc(){


    // queuing tasks
    // ros::Rate
    updateLoop();

    // Sensor update check
    sensorUpdate();

    // Update the pose and navigation parameters
    navUpdate();

    /*
    if(// senors alive) {
        // Update the the current mission task and calculate the desired control forces
        missionUpdate();

        // Move to the next task in the queue if the current task has just completed
        updateCurrentTask();

        // Send the thruster commands to the motor controller
        sendMotorCommands();

    } else{
        ROS_ERROR("BaeBotMaster: lidar, imu or obs_list not alive, skipping mission/motor update");
    }
`*/
    // Publishes summary pose messages
    publishPoseMessages();

    // Publish RViz markers

}

void BaeBotMaster::updateLoop(){

    updateDt();

}

void BaeBotMaster::updateDt(){



}
/**
*
* Update the pose structure with all the navigation information from the sensors
*
*/
void BaeBotMaster::navUpdate(){



}
/**
*
* Get status of all sensors and make sure they are still alive
*
*/
void BaeBotMaster::sensorUpdate(){



}

void BaeBotMaster::missionUpdate(){



}

void BaeBotMaster::updateCurrentTask(){



}

void BaeBotMaster::sendMotorCommands(){



}

void BaeBotMaster::publishPoseMessages(){



}


// SETTERS
void BaeBotMaster::setNewPose_xy( Point2D bb_goal ){
    poseDmd.x = bb_goal.x;
    poseDmd.y = bb_goal.y;
}

void BaeBotMaster::setNewPose( POSE poseGoal ){
    poseDmd.x = poseGoal.x;
    poseDmd.y = poseGoal.y;
    poseDmd.theta = poseGoal.theta;
}


// UTIL FUNCTINS
double BaeBotMaster::dist_to_point( Point2D point ){
 return sqrt( pow( point.x - pose.x, 2) + pow( point.y - pose.y, 2) );
}

double BaeBotMaster::dist_to_pose( POSE poseDmd ){
 return sqrt( pow( poseDmd.x - pose.x, 2) + pow( poseDmd.y - pose.y, 2) );
}
