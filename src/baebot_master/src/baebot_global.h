#ifndef BAEBOT_GLOBAL_H_INCLUDED
#define BAEBOT_GLOBAL_H_INCLUDED



#define DEG2RAD   M_PI / 180.0


// MISSION MODE
enum MISSION_MODE { MISSION_STOPPED = 0, MISSION_RUNNING = 1, MISSION_COMPLETED = 2, MISSION_ABORTED = 3, MISSION_PAUSED = 4, AWAITING_MISSION = 5, MISSION_DO_A_SPIN = 6, MISSION_GO_STRAIGHT = 7};

// CONTROL MODE
enum CONTROL_MODE { CONTROL_ESTOP = 0, CONTROL_JOYSTICK = 1, CONTROL_AUTONOMOUS = 2};


// SENSOR STATUS
typedef struct {
    bool cameraAlive = false;
    bool lidarAlive = false;
    bool poseAlive = false;

} SENSOR_STATUS;

#define LOOP_RATE 30

typedef struct {

    double x = 0;       // (m)
    double y = 0;       // (m)
    double theta = 0;   // (rads)

    double qx = 0;
    double qy = 0;
    double qz = 0;
    double qw = 1;

    double v = 0;
    double w = 0;
    double velX = 0;    // (m/s)
    double velY = 0;    // (m/s)

} POSE;


typedef struct {

    double x = 0;
    double y = 0;

} Point2D;





#endif // BAEBOT_GLOBAL_H_INCLUDED
