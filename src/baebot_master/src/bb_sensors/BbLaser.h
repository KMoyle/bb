#ifndef BBLASER_H_INCLUDED
#define BBLASER_H_INCLUDED

#include "ros/ros.h"
#include "../BaeBotMaster.h"

#define NUM_LASER_POINTS 2172






typedef struct {
    double range[NUM_LASER_POINTS];
	double	maxRange;
	int		maxInd;
	double	minRange;
	int		minInd;
	int		beamID;
	double 	beam_angle;
}lidarRing;

typedef struct {

    unsigned long seq_no;
    std::vector<lidarRing> ring;
    std::vector<float> bearing; //degrees

}lidarArray;



#endif // BAEBOTMASTER_H_INCLUDED
