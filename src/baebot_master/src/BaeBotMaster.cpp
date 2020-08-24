#include "BaeBotMaster.h"
#include "baebot_global.h"



BaeBotMaster::BaeBotMaster(ros::NodeHandle *nh ){





};

BaeBotMaster::~BaeBotMaster(){





};


double BaeBotMaster::dist2Point( POSE pose, POSE poseDmd ){
 return sqrt( pow( pose.x - poseDmd.x, 2) + pow( pose.y - poseDmd.y, 2) );
}
