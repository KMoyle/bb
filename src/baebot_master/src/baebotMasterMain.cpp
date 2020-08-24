#include "BaeBotMaster.h"
#include "baebot_global.h"
#include "ros/ros.h"

#include <boost/thread.hpp>
#include <boost/date_time.hpp>


int main( int argc, char **argv){

    ros::init(argc, argv, "baebotMasterMain");
    ros::NodeHandle nh("baebot_master");


   /*
    *   Initialise the Main Bot Controller
    */
    BaeBotMaster *baebotMaster = new BaeBotMaster( &nh );




    ros::spin();


    delete baebotMaster;

    return 0;


}


