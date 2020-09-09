#include "BbImage.h"





void BaeBotMaster::cameraImageCallback(const sensor_msgs::Image::ConstPtr& msg_in){

    timeSinceLastCameraUpdate =  0.0;
    sensor_status.cameraAlive = true;

    // Declare opencv mat pointer
    cv_bridge::CvImagePtr cv_ptr;
    // Try to convert recieved sensor_msgs/Image to opencv
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg_in, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }




};
