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

    // Map points
    std::pair<double, double> P5 = {0, 0};
    std::pair<double, double> P1 = {0.5, 0.5};
    std::pair<double, double> P2 = {-0.5, 0.5};
    std::pair<double, double> P3 = {-0.5, -0.5};
    std::pair<double, double> P4 = {0.5, -0.5};

    // Map points
    baebotMaster->goto_points.push_back(P5);
    baebotMaster->goto_points.push_back(P4);
    baebotMaster->goto_points.push_back(P3);
    baebotMaster->goto_points.push_back(P2);
    baebotMaster->goto_points.push_back(P1);
    baebotMaster->goto_points.push_back(P5);
    baebotMaster->goto_points.push_back(P4);
    baebotMaster->goto_points.push_back(P3);
    baebotMaster->goto_points.push_back(P2);
    baebotMaster->goto_points.push_back(P1);

    //commit for change!



    /*
    *   Initialise Threads
    */
    // Control threads
    // Planner threads
        baebotMaster->motorDmd_pub = nh.advertise< geometry_msgs::Twist >( "cmd_vel", 1 );



        boost::thread controlLoopThread( boost::bind( &BaeBotMaster::controlLoopFunc, baebotMaster ) );

        ROS_INFO("main: starting controller thread");
        controlLoopThread.join();




    return 0;


}


