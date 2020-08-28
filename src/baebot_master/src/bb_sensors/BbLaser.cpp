#include "ros/ros.h"
#include "BbLaser.h"





void BaeBotMaster::rpLidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan ){

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


       std::cout << "\nx= " << transformStamped.transform.translation.x ;
       std::cout << "\ny= " <<transformStamped.transform.translation.y ;
       std::cout << "\nz= " <<transformStamped.transform.translation.z ;


};
