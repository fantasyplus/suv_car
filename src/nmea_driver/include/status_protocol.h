#pragma once

#pragma pack(push, 1)

// broadcast at port 51000 UDP at 10Hz
typedef struct
{
    unsigned int stamp;
    unsigned char mode;     // 8bit 1-stop; 1-charging; 2-teleop; 3-auto; 4-follow code
    unsigned char gear;     // 0-stop; 1-forward; 2-backward
    unsigned char throttle; // dddd as dd.dd%
    unsigned char breaks;   // dddd as dd.dd%
    unsigned char speed;    // dddd as dd.dd km/h
    unsigned char alpha;    // steering angle dddd as -24.00~24.00 degree
    unsigned char battery;  // -1 as error , dddd as dd.dd%

    short voltage; // dddd as dd.dd V
    short current; // dddd as dd.dd A
} ChasisStatus;

// broadcast at port 51001 UDP at 5Hz
typedef struct
{
    unsigned int stamp;
    unsigned char nav_status; // 0-error; 1-normal
    unsigned char gps_status; // 0-error; 1-point; 2-difference
    float yaw;                // degree
    double lon;               // degree
    double lat;               // degree
} NavStatus;

// broadcast at port 51002 UDP at 10Hz
typedef struct
{
    unsigned int stamp;
    unsigned char mode;  // 0-quit; 1-teleop
    unsigned char focal; // cm
    unsigned char flash; // light
    char pitch;          // degree
    char yaw;            // degree
} PtzStatus;

// broadcast at port 51003 UDP at 10Hz
typedef struct
{
    unsigned int stamp;
    unsigned char mode;  // 8bit 1-stop; 1-charging; 2-teleop; 3-auto; 4-follow code
    unsigned char alpha; // steering angle dddd as -24.00~24.00 degree
    unsigned char speed; // dddd as dd.dd km/h
} MotionStatus;
#pragma pack(pop)