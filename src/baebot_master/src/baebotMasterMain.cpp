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
    baebotMaster->im_alive_pub = nh.advertise<std_msgs::String>( "BB_TALKS", 1 );

    std_msgs::StringPtr  str(new std_msgs::String);
    str->data = "IM ALIVE";




    /*
    *   Initialise Threads
    */
    // Control threads
    // Planner threads

    while(ros::ok()){

        baebotMaster->im_alive_pub.publish(str);

        ros::spinOnce();

        baebotMaster->r.sleep();



    }


    delete baebotMaster;

    return 0;


}


