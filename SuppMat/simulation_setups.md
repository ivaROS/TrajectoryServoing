# Simulation Setups
This file provides details about the setup of simulation experiments. All simulations are tested on i7-8700 (single thread passmark of 2669; multi-thread score of 13053)

## Simulation Environment

We build a simulation environment in Gazebo and spawn a Turtlebot as the non-holonomic robot. A simulated stereo camera with 0.07m baseline.

<p float="left">
  <img src="https://github.com/ivaROS/TrajectoryServoing/blob/main/SuppMat/assets/figs/tsrb_env.png" width = 30% height = 55% />
  <img src="https://github.com/ivaROS/TrajectoryServoing/blob/main/SuppMat/assets/figs/turtlebot_sim.png" width = 22% height = 55% /> 
</p>

## Testing Trajectory Shapes

There are 5 paths loosely based on Dubins paths. The average trajectory lengths are ∼4m. They are designed to ensure that sufficient feature points, visible in the first frame, remain visible along the entirety of the path.

<p float="left">
  <img src="https://github.com/ivaROS/TrajectoryServoing/blob/main/SuppMat/assets/figs/short_trajs.png" width = 30% height = 55% />
  <img src="https://github.com/ivaROS/TrajectoryServoing/blob/main/SuppMat/assets/figs/long_trajs.png" width = 30% height = 55% />
</p>

## SLAM Stack

A [Good Feature (GF) ORB-SLAM system](https://www.semanticscholar.org/paper/Good-Feature-Matching%3A-Toward-Accurate%2C-Robust-With-Zhao-Vela/da7817d3ed68d5cfbf47f89c39a8042896cae2c1) is used to estimate camera poses. It is configured to work with a stereo camera and integrated into a loosely coupled, [visual-inertial (VI) system] (https://github.com/ethz-asl/ethzasl_msf) based on a multi-rate filter to form a VI-SLAM system.

<p float="left">
  <img src="https://github.com/ivaROS/TrajectoryServoing/blob/main/SuppMat/assets/figs/feature.png" width = 30% height = 55% />
  <img src="https://github.com/ivaROS/TrajectoryServoing/blob/main/SuppMat/assets/figs/feature2.png" width = 30% height = 55% /> 
  <img src="https://github.com/ivaROS/TrajectoryServoing/blob/main/SuppMat/assets/figs/feature3.png" width = 30% height = 55% /> 
</p>

## Frame-by-frame Stereo Feature Tracking System for VS+

We also use a [frame-by-frame stereo feature tracking system](https://github.com/ivaROS/stereoFeatureTracking.git) to show the importance of V-SLAM. The system does not have mapping for feature matching with historical features.

