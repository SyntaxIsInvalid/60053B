#pragma once

// uncomment/comment the define depending on which robot

// #define ROBOT_TEST_DRIVE
#define ROBOT_COMPETITION

// Set feature flags based on robot selection
#if defined(ROBOT_TEST_DRIVE)
    #define HAS_INTAKE 0
    #define HAS_PNEUMATICS 0
    #define USE_ROTATION_TRACKER 0
#elif defined(ROBOT_COMPETITION)
    #define HAS_INTAKE 1
    #define HAS_PNEUMATICS 1
    #define USE_ROTATION_TRACKER 1
#else
    #error "No robot configuration selected!"
#endif