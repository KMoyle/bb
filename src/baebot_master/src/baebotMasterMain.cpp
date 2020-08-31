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
    *   Subscriber setup
    */


    /*
    *   Publisher setup
    */





    /*
    *   Initialise Threads
    */
    // Control threads
    // Planner threads

    while(ros::ok()){

        ros::spinOnce();

        baebotMaster->r.sleep();

        //ROS_WARN("SPINNING");
    }



    return 0;


}


