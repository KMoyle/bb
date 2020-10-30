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
        //ctor
        A_Star_Path_Planner( std::string name );

        //Callback functions
        void pathPlannerCallBack( const baebot_path_planner::PathPlannerGoal::ConstPtr& );
        void mapCallBack( const nav_msgs::OccupancyGrid& );

        //helper functions
        inline float return_g_score( MapCell* ,unsigned int );
        inline float return_h_score( MapCell* );
        void add_neighbours( MapCell* );
        MapCell *get_best_neighbour( );
        void compute_path( MapCell* );
        inline void setNeighbour( double, double );

        //dtor
        ~A_Star_Path_Planner();

        //Occ Grid members
        ros::Subscriber map_sub;
        nav_msgs::OccupancyGrid map_;

        //Action server vars
        actionlib::SimpleActionServer<baebot_path_planner::PathPlannerAction> as_;
        std::string action_name_;
        baebot_path_planner::PathPlannerFeedback feedback_;
        baebot_path_planner::PathPlannerResult result_;

        //A* members
        MapCell * start_{nullptr};
        MapCell * goal_{nullptr};




    private:

        ros::NodeHandle n_;

        bool map_ok_;

        const int map_width_{50};
        const int map_height_{50};

        bool found_goal;
        bool DEBUG{false};

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
