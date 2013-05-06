#ifndef CONFIG_H
#define CONFIG_H

// Comment out this line if your application has a QtGui
#define USE_QTGUI

#define OFFLINEMODE
// #define DEBUG_CUTRE



#define PARTICLES 200
#define VAR_CONSTANT
#define VARIANCE_R 0.06
#define VARIANCE_T 10
#define ANNEALING 0.6
#define ITERS 5
// #define ANNEALING 0.8
// #define ITERS 7
#define PARTICLE_DISTANCE_THRESHOLD 50
#define DOWNSAMPLE_INPUT 80
#define DOWNSAMPLE_VIRTUAL 90
#define PARTICLE_OPTIMIZATION_MIN 0
#define PARTICLE_OPTIMIZATION_MAX 200
#define DISTANCE_FUNCTION 2

#define T_X 0.
#define T_Y 0.
#define T_Z 0.
#define R_X 0.
#define R_Y 0.
#define R_Z 0.

#define INPUT_FILE "dataR3.pcd"
#define VIRTUAL_FILE "dataV3.pcd"
#define MUL_SCALE_FILE 1000.

// 	readPCD("pcd/00003_real.pcd", cloud_input);        // *1
// 	readPCD("pcd/00003_virtual.pcd", cloud_virtual);   // *1
// 	readPCD("cloudReal.pcd", cloud_input);             // *1
// 	readPCD("cloudVirtual.pcd", cloud_virtual);        // *1
// 	readPCD("real1.pcd", cloud_input);                 // *1000
// 	readPCD("virtual1.pcd", cloud_virtual);            // *1000
// 	readPCD("dataR1.pcd", cloud_input);                // *1000
// 	readPCD("dataV1.pcd", cloud_virtual);              // *1000
// 	float scale = 1000.;


#define PROGRAM_NAME    "gualzru_unknown"
#define SERVER_FULL_NAME   "RoboCompGualzruBehaviorUnknown::gualzru_unknown"

#endif
