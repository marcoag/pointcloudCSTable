#ifndef CONFIG_H
#define CONFIG_H

#define PARTICLES 200
#define ANNEALING 0.6
#define ITERS 5
#define DISTANCE_THRESHOLD 0.05

#define DOWNSAMPLE_INPUT 0.08
#define DOWNSAMPLE_VIRTUAL 0.09

#define PARTICLE_OPTIMIZATION_MIN 0
#define PARTICLE_OPTIMIZATION_MAX 0.2
#define DISTANCE_FUNCTION 2



const static float _data_matrix[7][6] = {
	// vT_X, vT_Y, vT_Z,  vR_X,  vR_Y,  vR_Z
	{   0.2,  0.2,  0.2,  0.02,  0.02,  0.02 }, // 1
	{   0.5,  0.2,  0.5,  0.01,  0.04444,  0.01 }, // 2
	{   0.2,  0.2,  0.2,  0.05,  0.15,  0.05 }, // 3
	{   0.2,  0.2,  0.2,  0.02,  0.02,  0.02 }, // 4
	{   0.2,  0.2,  0.2,  0.05,  0.15,  0.05 }, // 5
	{   0.1,  0.1,  1.5,  0.05,  0.05,  0.05 }, // 6
	{   0.2,  0.2,  0.2,  0.05,  0.15,  0.05 }, // 7
};

#endif
