#include "../include/A_Star_Path_Planner.h"


int main(int argc, char **argv) {

    ros::init(argc, argv, "a_star_planner");
    A_Star_Path_Planner bbPathPlanner( "Planner_Action" );
    ros::spin();

    return 0;
}


A_Star_Path_Planner::A_Star_Path_Planner( std::string name  )
        : map_ok_( false ),
         as_(n_, name, boost::bind( &A_Star_Path_Planner::pathPlannerCallBack, this, _1), false ),
         action_name_( name ) {

     map_sub = n_.subscribe("/map" ,10, &A_Star_Path_Planner::mapCallBack, this);


    ///init sizes of all a* vecs
    grid_.resize( map_height_ * map_width_ );
    parent_.resize( map_height_ * map_width_ );
    neighbour_ids_.resize( map_height_ * map_width_ );
    path_.reserve( map_height_ * map_width_ );
    open_set_.clear();
    closed_set_.clear();
    opened_.resize( map_height_ * map_width_ );
    fill(opened_.begin(), opened_.end(), false);
    closed_.resize( map_height_ * map_width_ );
    fill(closed_.begin(), closed_.end(), false);
    g_.resize( map_height_ * map_width_ );
    h_.resize( map_height_ * map_width_ );
    f_.resize( map_height_ * map_width_ );

    ///initialise grid
    for( unsigned int y =0; y < static_cast<unsigned int>( map_height_ ); y++){
        for( unsigned int x =0; x < static_cast<unsigned int>( map_width_ ); x++){
            grid_[ x + y * map_width_ ].x( x );
            grid_[ x + y * map_width_ ].y( y );
        }
    }

    goal_ = nullptr;
    start_ = nullptr;
    map_ = nav_msgs::OccupancyGrid();

    as_.start();


}

void A_Star_Path_Planner::pathPlannerCallBack( const baebot_path_planner::PathPlannerGoal::ConstPtr &goal ) {

    ROS_INFO("goal_x= %d and goal_y= %d ", goal->goal_x, goal->goal_y);
    ROS_INFO("start_x= %d and start_y= %d ", goal->start_x, goal->start_y);

    ///TODO transform goal into map coor
    goal_ = new MapCell( static_cast<unsigned int>( goal->goal_x/map_res_ ), static_cast<unsigned int>( goal->goal_y/map_res_ ) );
    start_ = new MapCell( static_cast<unsigned int>( goal->start_x/map_res_ ), static_cast<unsigned int>( goal->start_y/map_res_ ) );

    if ( map_ok_ ){
        ///init lists with start mapcell
        open_set_.push_back( start_ );
        opened_[ start_->get_x() + start_->get_y() * map_width_ ] = true;
        g_[ start_->get_x() + start_->get_y() * map_width_ ] = 0;
        h_[ start_->get_x() + start_->get_y() * map_width_ ] = return_h_score( start_ );
        f_[ start_->get_x() + start_->get_y() * map_width_ ] = g_[ start_->get_x() + start_->get_y() * map_width_ ] +  h_[ start_->get_x() + start_->get_y() * map_width_ ];


        ///start search
        while ( !open_set_.empty() ){

            ///finding cell with best f score
            MapCell *current = get_best_neighbour();
            float current_id_ = current->get_x() + current->get_y() * map_width_;

            if( 0 ) std::cout << "( " << current->get_x() << " , " << current->get_y() << " )" << std::endl;

            if ( current->get_x() == goal_->get_x() && current->get_y() == goal_->get_y() ){
                found_goal = true;
                std::cout << " FOUND PATH " << std::endl;
                compute_path( current );
                break;
            }
            ///remove current point from opened set and place in closed set and set closed to true
            closed_set_.push_back( open_set_.front() );
            closed_[ current_id_ ] = true;

            ///get valid neighbours of current i.e. within bounds
            neighbour_ids_.clear();
            add_neighbours( current );

            ///loop through neighbours
            for( auto itn = neighbour_ids_.begin() ; itn != neighbour_ids_.end(); itn++ ){

                ///firstly compute, cost = g(current) + distance(current,neighbour)
                float cost = g_[ current_id_ ] + return_g_score( current, *itn );

                if( DEBUG )std::cout << "neighbour ->  ( " <<  grid_[*itn].get_x() << " , " <<  grid_[*itn].get_y() << " )" <<  "\told g= " <<  g_[ *itn ] << "\tcost= " << cost << "\n" << std::endl;

                if( closed_[*itn] && cost < g_[ *itn ] ){
                     g_[ *itn ] = cost;
                     f_[ *itn ] = g_[ *itn ] + h_[ *itn ];
                    closed_[*itn] = false;
                    opened_[*itn] = true;
                    if( DEBUG )std::cout << "neighbours parent was ( " << parent_[ *itn ]->get_x() << " , " << parent_[ *itn ]->get_y() << " )" << std::endl;
                    parent_[ *itn ] = &grid_[current_id_];
                    if( DEBUG )std::cout << "neighbours parent is now ( " << parent_[ *itn ]->get_x() << " , " << parent_[ *itn ]->get_y() << " )" << std::endl;

                }else if ( opened_[*itn] && cost < g_[ *itn ] ){
                    g_[ *itn ] = cost;
                    f_[ *itn ] = g_[ *itn ] + h_[ *itn ];
                    if( DEBUG )std::cout << "neighbours parent was ( " << parent_[ *itn ]->get_x() << " , " << parent_[ *itn ]->get_y() << " )" << std::endl;
                    parent_[ *itn ] = &grid_[current_id_];
                    if( DEBUG )std::cout << "neighbours parent is now ( " << parent_[ *itn ]->get_x() << " , " << parent_[ *itn ]->get_y() << " )" << std::endl;

                }else if( !opened_[*itn] && !closed_[*itn] ){
                    open_set_.push_back( &grid_[ *itn ] );
                    opened_[ *itn  ] = true;
                    g_[ *itn ] = cost;
                    h_[ *itn ] = return_h_score( &grid_[ *itn ] );
                    f_[ *itn ] = g_[ *itn ] + h_[ *itn ];

                }
            }

        }

    } else {
        ROS_WARN( "map not updated yet");
        as_.setAborted();
        delete goal_;
        delete start_;
    }
}
void A_Star_Path_Planner::mapCallBack( const nav_msgs::OccupancyGrid &occ_map ){

    map_ok_ = true;
    map_ = occ_map;

    ROS_INFO("\nMAP Details-> Width= %d\nHeight= %d\nResolution= %f\nOrigin of cell(0,0) is ( %f , %f )", occ_map.info.width, occ_map.info.height, occ_map.info.resolution, occ_map.info.origin.position.x, occ_map.info.origin.position.y);


}


MapCell *A_Star_Path_Planner::get_best_neighbour( ){

    MapCell *best;
    unsigned int current_id = 0;
    std::list<MapCell *>::iterator best_itf;
    ///iterating through the list to find the mapcell with the best f score
    for( auto itf = open_set_.begin(); itf != open_set_.end(); itf++){
        if ( itf == open_set_.begin() || ( ( f_[(*itf)->get_x() + (*itf)->get_y() * map_width_] ) < f_[current_id]  )  ){
            current_id = (*itf)->get_x() + (*itf)->get_y() * map_width_;
            best = &grid_[current_id];
            best_itf = itf;
        }
    }
    open_set_.erase( best_itf );
    return best;

}


inline float A_Star_Path_Planner::return_g_score( MapCell *cmp, unsigned int neighbour_id ){

    float dx = static_cast<float>( cmp->get_x() ) - static_cast<float>(grid_[neighbour_id].get_x());
    float dy = static_cast<float>( cmp->get_y() ) - static_cast<float>(grid_[neighbour_id].get_y());

    return  ( dx * dx ) + ( dy * dy ) ;
}
inline float A_Star_Path_Planner::return_h_score( MapCell *mp ){

    float dx = static_cast<float>( mp->get_x() ) - static_cast<float>(goal_->get_x());
    float dy = static_cast<float>( mp->get_y() ) - static_cast<float>(goal_->get_y());

    return ( dx * dx ) + ( dy * dy ) ;

}

void A_Star_Path_Planner::add_neighbours( MapCell *cmp ){
    /// TODO add robustness for inf or obs for real maps
    unsigned int neighbour_id;
    unsigned int current_id_ = cmp->get_x() + cmp->get_y() * map_width_;


    for ( int y = -1; y < 2; y++){
        for ( int x = -1; x < 2; x++ ){

            if ( ( cmp->get_x() > 0 ) &&  cmp->get_y() > 0 && ( cmp->get_x() + 1 ) < map_width_ && ( cmp->get_y() + 1 ) < map_height_ && !closed_[ ( cmp->get_x() + (x) ) + ( cmp->get_y() + (y) ) * map_width_] && map_.data[ ( cmp->get_x() + (x) ) + ( cmp->get_y() + (y) ) * map_width_] <= 0 ){
                neighbour_id = ( cmp->get_x() + (x) ) + ( cmp->get_y() + (y) ) * map_width_;
                setNeighbour( neighbour_id, current_id_ );

            } else if ( ( cmp->get_x() == 0 ) && ( cmp->get_y() > 0 ) && !closed_[ ( cmp->get_y() + (y) ) * map_width_] && map_.data[ ( cmp->get_y() + (y) ) * map_width_] <= 0 ){
                neighbour_id =  ( cmp->get_y() + (y) ) * map_width_;
                setNeighbour( neighbour_id, current_id_ );

            } else if ( ( cmp->get_x() > 0 ) && ( cmp->get_y() == 0 ) && !closed_[ ( cmp->get_x() + (x) ) ] && map_.data[ ( cmp->get_x() + (x) ) ] <= 0 ){
                neighbour_id = ( cmp->get_x() );
                setNeighbour( neighbour_id, current_id_ );

            } else if ( ( (cmp->get_x() + 1 ) ==  map_width_ ) && ( cmp->get_y() + 1 ) < map_height_ && !closed_[ ( cmp->get_x() ) +  ( cmp->get_y() + 1 ) * map_width_] && map_.data[ ( cmp->get_x() ) +  ( cmp->get_y() + 1 ) * map_width_ ] <= 0 ){
                neighbour_id = ( ( cmp->get_x() ) +  ( cmp->get_y() + 1 ) * map_width_ );
                setNeighbour( neighbour_id, current_id_ );

            } else if ( ( (cmp->get_x() + 1 ) <  map_width_ ) && ( cmp->get_y() + 1 ) == map_height_ && !closed_[ ( cmp->get_x() + 1 ) +  ( cmp->get_y()  ) * map_width_] && map_.data[ ( cmp->get_x() + 1 ) +  ( cmp->get_y()  ) * map_width_ ] <= 0 ){
                neighbour_id = ( ( cmp->get_x() + 1 ) +  ( cmp->get_y()  ) * map_width_ );
                setNeighbour( neighbour_id, current_id_ );

            } else if (  ( cmp->get_x() == 0 ) && ( cmp->get_y() == 0 ) && x >= 0 && y >= 0 && !closed_[ ( cmp->get_x() + (x) ) + ( cmp->get_y() + (y) ) * map_width_] && map_.data[ ( cmp->get_x() + (x) ) + ( cmp->get_y() + (y) ) * map_width_] <= 0 ){
                neighbour_id = ( cmp->get_x() + (x) ) + ( cmp->get_y() + (y) ) * map_width_;
                setNeighbour( neighbour_id, current_id_ );
            }

        }
    }

}
inline void A_Star_Path_Planner::setNeighbour( double nid, double cid ){
    ///add neighbour id to new neighbours vec
    neighbour_ids_.push_back( nid );
    ///if not opened add its parent node
    if ( !opened_[ nid] ) parent_[ nid ] = &grid_[ cid] ;

}



void A_Star_Path_Planner::compute_path( MapCell* current ){

    ///init path with current (last) point
    path_.push_back( current );

    ///loop through the parent vector and add all values to path
    while( current->get_x() != start_->get_x() &&  current->get_y() != start_->get_y() ){
        //std::cout << "next in path -- >> " << "( " << current->get_x() << " , " << current->get_y()<< " )" << std::endl;
        path_.push_back( parent_[ current->get_x() + current->get_y() * map_width_ ] );
        current = parent_[ current->get_x() + current->get_y() * map_width_ ];
    }
    ///reverse path before sending
    std::reverse(path_.begin(),path_.end());
    /// Assign path the as_ result msg
    for( auto it : path_ ) {
        result_.path_x.push_back( it->get_x() );
        result_.path_y.push_back( it->get_y() );
    }
    ///Send result via as_
    as_.setSucceeded( result_ );

    ///  USED TO PRINT & VALIDATE path_
    if ( DEBUG ){
        for ( auto itp : path_){
            std::cout << "( " << itp->get_x() << " , "<< itp->get_y() << " )" << std::endl;
        }
    }

}



A_Star_Path_Planner::~A_Star_Path_Planner()
{
    //dtor
}

