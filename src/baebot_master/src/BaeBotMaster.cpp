#include "BaeBotMaster.h"
#include "baebot_global.h"



BaeBotMaster::BaeBotMaster(ros::NodeHandle *nh ) :
                it_(*nh)
{



        //laser_sub = nh->subscribe<sensor_msgs::LaserScan>("/rplidar_scan" , 1, &BaeBotMaster::rpLidarCallback, this);
        laser_sub = nh->subscribe<sensor_msgs::LaserScan>("/scan" , 1, &BaeBotMaster::rpLidarCallback, this);
        //laser_sub = nh->subscribe<sensor_msgs::MultiEchoLaserScan>("/horizontal_laser_2d" , 1, &BaeBotMaster::rpLidarCallback, this);
        image_sub = it_.subscribe("/camera/image_raw" , 1, &BaeBotMaster::cameraImageCallback, this);


        ROS_WARN("ctor");
};

BaeBotMaster::~BaeBotMaster(){





};

void BaeBotMaster::controlLoopFunc(){

    int	controlLoopCnt = 0;
    ros::Rate loop_rate(LOOP_RATE);


    while ( ros::ok() ) {

        // queuing tasks
        // ros::Rate
        updateLoop();

        if ( 0 ) ROS_INFO( "Control Loop Cnt: %d", controlLoopCnt++ );

        ros::spinOnce();
        loop_rate.sleep();
    }



}

void BaeBotMaster::updateLoop(){

    updateDt();

        // Sensor update check
    sensorUpdate();

    // TODO -- controller update

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

void BaeBotMaster::updateDt(){

     ros::Time currentTime = ros::Time::now();

     if ( lastUpdateTime == NULL ) {
          dt = ros::Duration( 1.0 / sampleRate );
     } else {
          dt = currentTime - *lastUpdateTime;
     }
     lastUpdateTime = new ros::Time( currentTime.sec, currentTime.nsec );

     if ( dt > ros::Duration( 1.5 / sampleRate ) ) {
          ROS_WARN( "Time between updates (%2f) was greater than 1.5 times the requested time (%2f)!", dt.toSec(), (1.0 / sampleRate) );
     }


}
/**
*
* Update the pose structure with all the navigation information from the sensors
*
*/
void BaeBotMaster::navUpdate(){

    double xdot, ydot, thetadot;

    // TODO -- function that takes the WHEEL encoder info and outputs the vw

    // using the velocity and current angle to work out the change in x, y & theta
    xdot = pose.v * cos( pose.theta );
    ydot = pose.v * sin( pose.theta );
    thetadot = pose.w;

    // updating the new pose information
    pose.x = pose.x + xdot*(dt.toSec());
    pose.y = pose.y + ydot*(dt.toSec());
    pose.theta = pose.theta + thetadot*(dt.toSec());





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
