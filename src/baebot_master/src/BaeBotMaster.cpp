#include "BaeBotMaster.h"
#include "baebot_global.h"



BaeBotMaster::BaeBotMaster(ros::NodeHandle *nh ) :
                it_(*nh)
{
        // Subscribers
        laser_sub = nh->subscribe<sensor_msgs::LaserScan>("/scan" , 1, &BaeBotMaster::rpLidarCallback, this);
        pose_sub = nh->subscribe<nav_msgs::Odometry>("/odom" , 1, &BaeBotMaster::bbPoseCallback, this);
        image_sub = it_.subscribe("/camera/rgb/image_raw" , 1, &BaeBotMaster::cameraImageCallback, this);
        //image_sub = it_.subscribe("/BaeBot/camera/image_raw" , 1, &BaeBotMaster::cameraImageCallback, this);
        battery_sub = nh->subscribe("/battery" , 1, &BaeBotMaster::batteryStateCallback, this);

        // MISSION STATUS INIT
        mission_status = AWAITING_MISSION;

        // Intitalising the sensor last updates
        timeSinceLastLidarUpdate = sensorTimeOut;
        timeSinceLastCameraUpdate = sensorTimeOut;
        timeSinceLastPoseUpdate = sensorTimeOut;

        ROS_INFO("ctor");
};

BaeBotMaster::~BaeBotMaster(){


        ROS_INFO("dtor");


};

void BaeBotMaster::controlLoopFunc(){

    int	controlLoopCnt{0};
    ros::Rate loop_rate( r );


    while ( ros::ok() ) {

        // queuing tasks
        //ros::Rate
        updateLoop();

        if ( DEBUG ) ROS_INFO( "Control Loop Cnt: %d", controlLoopCnt++ );

        ros::spinOnce();
        loop_rate.sleep();
    }



}

void BaeBotMaster::pathPlannerLoopFunc(){

    int	pathLoopCnt{0};
    ros::Rate loop_rate( r );

    actionlib::SimpleActionClient<baebot_path_planner::PathPlannerAction> ac_( "Planner_Action" );

    if ( DEBUG_PLANNER && test_new_path ){
        pathgoal.goal_x = 3;
        pathgoal.goal_y = 3;

        pathgoal.start_x = static_cast<unsigned int>( pose.x );
        pathgoal.start_y = static_cast<unsigned int>( pose.y );

        ac_.sendGoal( pathgoal );
        need_new_path = true;
        test_new_path = false;
    }

    while( ros::ok() ){

        if ( DEBUG ) ROS_INFO( "Path Loop Cnt: %d", pathLoopCnt++ );

        if ( need_new_path ){

            if ( ac_.isServerConnected() ){
                if ( DEBUG_PLANNER ) ROS_INFO( "BaeBotMaster: Action server connected, sending goal" );

                ac_.sendGoal( pathgoal );
                waiting_for_path = true;
                need_new_path = false;

            }else{
                ROS_WARN( "BaeBotMaster: UNABLE TO CONNECT TO PATH PLANNER SERVER" );
            }
         }

         if( ac_.isServerConnected() && waiting_for_path ) {

            if( ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED )
                pathResult = ac_.getResult();
                waiting_for_path = false;
                  if ( DEBUG_PLANNER )ROS_INFO( "BaeBotMaster: Result recieved from planner" );

        }else{
                 if ( DEBUG_PLANNER )ROS_INFO( "BaeBotMaster: waiting for path from planner" );

        }



        ros::spinOnce();
        loop_rate.sleep();
    }


}

void BaeBotMaster::updateLoop(){

    updateDt();

    /// Sensor update check
    //sensorUpdate();

    ///ODOM TEST
    //doASpin();
    //goStraight();

    /// Update the pose and navigation parameters
    navUpdate();

    if( sensor_status.poseAlive && sensor_status.cameraAlive && sensor_status.lidarAlive ) {
        /// Update the the current mission task and calculate the desired control forces
        missionUpdate();

        /// Move to the next task in the queue if the current task has just completed
        updateCurrentTask();

        /// Send the thruster commands to the motor controller
        publishMotorCommands();

    } else{
        mission_status = MISSION_STOPPED;
        ROS_ERROR("BaeBotMaster: lidar, camera or pose not alive, skipping mission/motor update");
    }

    /// Publishes summary pose messages
    publishPoseMessages();

    /// TODO Publish RViz markers

}

void BaeBotMaster::updateDt(){

     ros::Time currentTime = ros::Time::now();

     if ( lastUpdateTime == NULL ) {
          dt = ros::Duration( 1.0 / sampleRate );
     }else {
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

    static int num_wp{0};
    static int bread_crumb_counter{0};

    ros::Time currentTime = ros::Time::now();

    /// TODO -->> LOGIC TO DETERMINE WHAT CONTROLLER IM USING
    // Initial waypoint
    if ( we_are_off ){
        ROS_INFO("Off on our first mission");
        mission_status = MISSION_RUNNING;
        poseDmd.x = goto_points.front().first;
        poseDmd.y = goto_points.front().second;
        goto_points.pop();
        we_are_off = true;
    }
    if ( !goto_points.empty() && bread_crumb_counter++ > (int) ( 2*sampleRate )){
        num_wp++;
        ROS_INFO("ONTO NEXT WAY POINT - %d", num_wp);
        poseDmd.x = goto_points.front().first;
        poseDmd.y = goto_points.front().second;
        ROS_INFO( " poseDmd= ( %f , %f )", poseDmd.x, poseDmd.y);
        goto_points.pop();
        bread_crumb_counter = 0;

    }


}
/**
*
* Get status of all sensors and make sure they are still alive
*
*/
void BaeBotMaster::sensorUpdate(){

    sensor_status.cameraAlive = ( (timeSinceLastCameraUpdate += dt.toSec() ) < sensorTimeOut );

    if ( DEBUG ) { ROS_INFO( "Time Since Last Camera Update = %f", timeSinceLastCameraUpdate); }

    sensor_status.lidarAlive = ( (timeSinceLastLidarUpdate += dt.toSec() ) < sensorTimeOut );

    if ( DEBUG ) { ROS_INFO( "Time Since Last Lidar Update = %f", timeSinceLastLidarUpdate); }

    sensor_status.poseAlive = ( (timeSinceLastPoseUpdate += dt.toSec() ) < sensorTimeOut );

    if ( DEBUG ) { ROS_INFO( "Time Since Last Pose Update = %f", timeSinceLastPoseUpdate); }



}

void BaeBotMaster::missionUpdate(){


    // MISSION SWITCH
    switch( mission_status ){

        case AWAITING_MISSION:
            motor_cmds_vw.first = 0;
            motor_cmds_vw.second = 0;
            if ( DEBUG_SM ) ROS_INFO("AWAITING_MISSION");
            break;

        case MISSION_RUNNING:
            motor_cmds_vw = baeBotControl.controllerPurePursuit( pose, poseDmd );
            //motor_cmds_vw = baeBotControl.controllerProportional( pose, poseDmd );
            if ( DEBUG_SM ) ROS_INFO("MISSION_RUNNING");
            break;

        case MISSION_COMPLETED:
            motor_cmds_vw.first = 0;
            motor_cmds_vw.second = 0;
            if ( DEBUG_SM ) ROS_INFO("MISSION_COMPLETED");
            break;

        case MISSION_PAUSED:
            motor_cmds_vw.first = 0;
            motor_cmds_vw.second = 0;
            if ( DEBUG_SM ) ROS_INFO("MISSION_PAUSED");
            break;

        case MISSION_STOPPED:
            motor_cmds_vw.first = 0;
            motor_cmds_vw.second = 0;
            if ( DEBUG_SM ) ROS_INFO("MISSION_STOPPED");
            break;

        case MISSION_DO_A_SPIN:
            motor_cmds_vw.first = 0;
            motor_cmds_vw.second = 1;
            if ( DEBUG_SM ) ROS_INFO("MISSION_DO_A_SPIN");
            break;

        case MISSION_GO_STRAIGHT:
            motor_cmds_vw.first = 0.05;
            motor_cmds_vw.second = 0;
            if ( DEBUG_SM ) ROS_INFO("MISSION_GO_STRAIGHT");
            break;
}


}

void BaeBotMaster::updateCurrentTask(){



}

void BaeBotMaster::publishMotorCommands( ){

    // ROS twist msg type
    geometry_msgs::Twist msg;
    // Filling the msg
    msg.linear.x = motor_cmds_vw.first;
    msg.linear.y = 0;
    msg.angular.z = motor_cmds_vw.second;
    // Publishing the msg
    motorDmd_pub.publish(msg);



}

void BaeBotMaster::publishPoseMessages(){



}

void BaeBotMaster::bbPoseCallback(const nav_msgs::Odometry::ConstPtr& msg){

    timeSinceLastPoseUpdate =  0.0;
    sensor_status.poseAlive = true;

    double roll, pitch, yaw;
    // Getting the Yaw info from  Quaternion
    tf::Quaternion q (msg->pose.pose.orientation.x,
                       msg->pose.pose.orientation.y,
                       msg->pose.pose.orientation.z,
                       msg->pose.pose.orientation.w);
    tf::Matrix3x3 m( q );
    m.getRPY(roll, pitch, yaw);

    // Updating the pose info from the base_control node
    pose.x = msg->pose.pose.position.x;
    pose.y = msg->pose.pose.position.y;
    pose.theta = yaw;
    pose.qx = msg->pose.pose.orientation.x;
    pose.qy = msg->pose.pose.orientation.y;
    pose.qz = msg->pose.pose.orientation.z;
    pose.qw = msg->pose.pose.orientation.w;
    pose.velX = msg->twist.twist.linear.x;
    pose.velY = msg->twist.twist.linear.y;


   if ( DEBUG_POSE ) ROS_INFO("Current Pose: X= %f     Y= %f     theta= %f", pose.x, pose.y, pose.theta);

}


void BaeBotMaster::batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg){

    if ( msg->voltage < 10.5 ){
        ROS_WARN("BATTERY VOLTAGE IS BELOW 10.5V, TIME TO RECHARGE");

    }
    if ( DEBUG ) { ROS_INFO("BATTERY VOLTAGE IS %2.2f", msg->voltage );}

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

double BaeBotMaster::dist_to_pose(  ){
 return sqrt( pow( poseDmd.x - pose.x, 2) + pow( poseDmd.y - pose.y, 2) );
}


void BaeBotMaster::doASpin(){

        mission_status = MISSION_DO_A_SPIN;

}

void BaeBotMaster::goStraight(){

        mission_status = MISSION_GO_STRAIGHT;

}

void BaeBotMaster::pathPlannerTest(){




}
