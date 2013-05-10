# Point Cloud Cognitive Subtraction

This repository holds the code related to the paper submited to ECMR 2013: "Cognitive Subtraction: a Tool to Support Autonomous Robot Perception".
Is is provided in order to compare how Cognitive Subtraction would work in the presence of errors with:
* Raw data
* Iterative Closest Points (ICP)
* The Annealed Particle Filter proposed in the paper

The following is an screenshot of the program:
![Figure](https://raw.github.com/ljmanso/pointCloudCS/master/example1.png "Figure 1")

## Compilation

$ cmake .

$ make

## Execution

$ ./cogSub *DATASET*

where *DATASET* is the example in the dataset to use.



