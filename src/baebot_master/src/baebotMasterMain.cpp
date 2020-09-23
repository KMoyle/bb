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


    double delta_dist = 0.1;
    std::pair<double, double> P_old({0,0});
    std::pair<double, double> P_new({0,0});


    //Filling (x,y) points for pp testing
    for ( int i =0; i < 5; i++ ){
        if( i == 0 ){
            for ( int k = 0; k < 5; k++ ){
                P_new.first = P_old.first + delta_dist;
                P_new.second = P_old.second + delta_dist;
                baebotMaster->goto_points.push(P_new);
                P_old = P_new;
            }
        }
        if( i == 1 ){
            for ( int k = 0; k < 10; k++ ){
                P_new.first = P_old.first - delta_dist;
                //P_new.second = P_old.second + delta_dist;
                baebotMaster->goto_points.push(P_new);
                P_old = P_new;
            }
        }
        if( i == 2 ){
            for ( int k = 0; k < 10; k++ ){
                P_new.first = P_old.first + delta_dist;
                P_new.second = P_old.second - delta_dist;
                baebotMaster->goto_points.push(P_new);
                P_old = P_new;
            }
        }
        if( i == 3 ){
            for ( int k = 0; k < 10; k++ ){
                P_new.first = P_old.first - delta_dist;
                //P_new.second = P_old.second - delta_dist;
                baebotMaster->goto_points.push(P_new);
                P_old = P_new;
            }
        }
         if( i == 4 ){
            for ( int k = 0; k < 5; k++ ){
                P_new.first = P_old.first + delta_dist;
                P_new.second = P_old.second + delta_dist;
                baebotMaster->goto_points.push(P_new);
                P_old = P_new;
            }
        }
    }

    /*
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
    */


    /*
    *   Initialise Threads
    */
    // Control threads
    // Planner threads
        baebotMaster->motorDmd_pub = nh.advertise< geometry_msgs::Twist >( "cmd_vel", 1 );



        boost::thread controlLoopThread( boost::bind( &BaeBotMaster::controlLoopFunc, baebotMaster ) );

        ROS_INFO("main: starting controller thread");
        controlLoopThread.join();


        delete baebotMaster;




    return 0;


}


