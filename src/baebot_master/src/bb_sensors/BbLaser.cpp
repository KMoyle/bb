#include "ros/ros.h"
#include "BbLaser.h"


void BaeBotMaster::rpLidarCallback( const sensor_msgs::LaserScan::ConstPtr& scan ){

     timeSinceLastLidarUpdate = 0.0;
     sensor_status.lidarAlive = true;

    //tf2_ros::Buffer tfBuffer;
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::TransformStamped transformStamped;



       try{
         transformStamped = tfBuffer.lookupTransform( "base_link", "laser",  ros::Time(0));
       }
       catch (tf2::TransformException &ex) {
         ROS_WARN("%s",ex.what());
       }



};

