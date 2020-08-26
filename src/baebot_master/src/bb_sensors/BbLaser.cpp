#include "ros/ros.h"
#include "BbLaser.h"





void BaeBotMaster::rpLidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan ){


   sensor_msgs::PointCloud cloud;

   projector.projectLaser(*scan, cloud);



};
