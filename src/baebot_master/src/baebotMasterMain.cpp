#include "BaeBotMaster.h"
#include "baebot_global.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <boost/thread.hpp>
#include <boost/date_time.hpp>


int main( int argc, char **argv){

    ros::init(argc, argv, "baebotMasterMain");
    ros::NodeHandle nh("baebot_master");

    /*
    *   Initialise the Main Bot Controller
    */
    BaeBotMaster *baebotMaster = new BaeBotMaster( &nh );

    /*
    *   Initialise Threads
    */
    boost::thread controlLoopThread( boost::bind( &BaeBotMaster::controlLoopFunc, baebotMaster ) );
    boost::thread pathPlannerLoodThread ( boost::bind( &BaeBotMaster::pathPlannerLoopFunc, baebotMaster ) );

    ROS_INFO("main: starting controller thread");
    controlLoopThread.join();
    pathPlannerLoodThread.join();

    delete baebotMaster;

    return 0;


}


