#ifndef A_STAR_PATH_PLANNER_H
#define A_STAR_PATH_PLANNER_H


#include "ros/ros.h"

#include <vector>
#include <list>
#include <cmath>
#include <iostream>
#include <algorithm>

#include <actionlib/server/simple_action_server.h>
#include <baebot_path_planner/PathPlannerAction.h>

#include <nav_msgs/OccupancyGrid.h>

#include "Map_Cell.h"



class A_Star_Path_Planner
{
    public:
        A_Star_Path_Planner( std::string name );

        void pathPlannerCallBack( const baebot_path_planner::PathPlannerGoal::ConstPtr );

        float return_g_score( MapCell* ,unsigned int );
        float return_h_score( MapCell* );

        void add_neighbours( MapCell* );
        MapCell *get_best_neighbour( );

        void compute_path( MapCell* );
        void print_map();

        ~A_Star_Path_Planner();


        actionlib::SimpleActionServer<baebot_path_planner::PathPlannerAction> as_;
        std::string action_name_;
        baebot_path_planner::PathPlannerFeedback feedback_;
        baebot_path_planner::PathPlannerResult result_;

        bool found_goal;
        bool DEBUG = true;

        MapCell * start_;
        MapCell * goal_;
        nav_msgs::OccupancyGrid map_;


    private:

        ros::NodeHandle n_;

        bool map_ok_;

        const int map_width_ = 100;
        const int map_height_= 100;


        // A star Search variables
        std::vector<MapCell> grid_; //main grid containing map cells
        std::vector<MapCell *> parent_; //layer array containing pointers to parent cell for each cell
        std::vector<MapCell *> path_; // layer array containing returned best path
        std::vector<unsigned int> neighbour_ids_; // layer array containing neihbours of current MapCell

        std::list<MapCell *> open_set_; //list of opened mapcells
        std::list<MapCell *> closed_set_; //list of closed mapcells

        std::vector<bool> opened_; //layer array contain opened flag for each map cell
        std::vector<bool> closed_; //layer array contain closed flag for each map cell

        std::vector<float> g_; //layer array containing g values for each map cell -->> dist covered
        std::vector<float> h_; //layer array containing h values for each map cell -->> dist to goal
        std::vector<float> f_; //layer array containing f values for each map cell -->> total cost




};

#endif // A_STAR_PATH_PLANNER_H
