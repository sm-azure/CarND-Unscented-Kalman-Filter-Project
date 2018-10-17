# Unscented Kalman Filter Project Writeup
Self-Driving Car Engineer Nanodegree Program

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

## Rubric - Compilation
No changes have been made to the CMakeLists.txt. Simple `cmake` and `make` should be enough to compile

## Accuracy 
* For dataset 1, the RMSE (X,Y, VX,VY) are 0.0719, 0.0845, 0.3566, 0.2608 which is under the expected .09, .10, 0.40, 0.30 values.

## Algorithmic correctness and efficiency
Code has been checked for algorithmic correctness. There does not seem to be any un-necessary code either. 
