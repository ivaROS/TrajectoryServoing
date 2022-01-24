# Real Experiment Setups
This file provides details about the setup of real experiments. All experiments are run on a [LoCoBot](http://www.locobot.org/) equipped with a RealSense stereo camera D435i and an Intel NUC (i5-7260U) (single thread passmark of 1991; multi-thread score of 3884)

<img src="https://github.com/ivaROS/TrajectoryServoing/blob/main/SuppMat/assets/figs/locobot.png" width = 30% height = 55% />

## Experiment Scenarios

The real experiments are performed in our lab area. There are no any artificial markers in the environments. A topview camera records the experiment trials. All short trajectory experiments can be included in the field of view of the camera. Blue box is the robot's start pose. Red box shows the end poses region of short trajectories. The green curve is a sample trajectory to track.
In the long trajectory experiments, due the long length of the trajectory and limited field of view, only end portion of the experiments are recorded. The same blue box as short trajectory experiment does not indicate the end pose of the robot.

<p float="left">
  <img src="https://github.com/ivaROS/TrajectoryServoing/blob/main/SuppMat/assets/figs/TMECH_real_exp.png" width = 30% height = 55% />
  <img src="https://github.com/ivaROS/TrajectoryServoing/blob/main/SuppMat/assets/figs/real_features.png" width = 30% height = 55% /> 
</p>

## Testing Trajectory Shapes

The same short trajectory templates are used. But in order to maintain feature tracking without triggering feature replenishment, they are scaled down to ∼2.4m. Two new long trajectory template are created.

<p float="left">
  <img src="https://github.com/ivaROS/TrajectoryServoing/blob/main/SuppMat/assets/figs/LS.png" width = 30% height = 55% />
  <img src="https://github.com/ivaROS/TrajectoryServoing/blob/main/SuppMat/assets/figs/LT.png" width = 30% height = 55% /> 
</p>

## [Short Distance Experiments Demos](https://youtu.be/haGQeROMAgo)

## [Long Distance Experiments Demos](https://youtu.be/ss49UZmLmeo)