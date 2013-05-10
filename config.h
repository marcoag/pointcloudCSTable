#ifndef CONFIG_H
#define CONFIG_H

#define PARTICLES 200
#define VAR_CONSTANT
#define ANNEALING 0.6
#define ITERS 5
#define PARTICLE_DISTANCE_THRESHOLD 0.05
#define DOWNSAMPLE_INPUT 0.08
#define DOWNSAMPLE_VIRTUAL 0.09
#define PARTICLE_OPTIMIZATION_MIN 0
#define PARTICLE_OPTIMIZATION_MAX 0.2
#define DISTANCE_FUNCTION 2

// #define vT_X 0.
// #define vT_Y 0.1
// #define vT_Z 0.5
// #define vR_X 0.0
// #define vR_Y 0.0
// #define vR_Z 0.0

const static float _data_matrix[6][6] = {
//vT_X, vT_Y, vT_Z, vR_X, vR_Y, vR_Z
{0, 0, 0, 0, 0, 0}, //1
{0, 0, 0, 0, 0, 0}, //2
{0, 0, 0, 0, 0, 0}, //3
{0, 0, 0, 0, 0, 0}, //4
{0, 0, 0, 0, 0, 0}, //5
{0, 0, 0, 0, 0, 0}  //6
};
  

/*
First data set:
_data_matrix[0][0]= 0; //vT_X
_data_matrix[0][1]= 0; //vT_Y
_data_matrix[0][2]= 0; //vT_Z
_data_matrix[0][3]= 0; //vR_X
_data_matrix[0][4]= 0; //vR_Y
_data_matrix[0][5]= 0; //vR_Z

First data set:
_data_matrix[1][0]= 0; //vT_X
_data_matrix[1][1]= 0; //vT_Y
_data_matrix[1][2]= 0; //vT_Z
_data_matrix[1][3]= 0; //vR_X
_data_matrix[1][4]= 0; //vR_Y
_data_matrix[1][5]= 0; //vR_Z

First data set:
_data_matrix[2][0]= 0; //vT_X
_data_matrix[2][1]= 0; //vT_Y
_data_matrix[2][2]= 0; //vT_Z
_data_matrix[2][3]= 0; //vR_X
_data_matrix[2][4]= 0; //vR_Y
_data_matrix[2][5]= 0; //vR_Z

First data set:
_data_matrix[3][0]= 0; //vT_X
_data_matrix[3][1]= 0; //vT_Y
_data_matrix[3][2]= 0; //vT_Z
_data_matrix[3][3]= 0; //vR_X
_data_matrix[3][4]= 0; //vR_Y
_data_matrix[3][5]= 0; //vR_Z*/

#endif
