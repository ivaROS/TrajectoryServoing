### Collect and build all packages required

__Note: you must have your ssh keys set up for this to work properly!__ Follow [this tutorial](https://help.github.com/articles/connecting-to-github-with-ssh/) to setup your ssh keys with github. 

Following is a general series of steps to accomplish the most common tasks with `wstool`.

1. Install the `wstool`:
```
sudo apt-get install python-rosdep  python-wstool  build-essential python-rosinstall-generator python-rosinstall
```

Note: All following commands should be run from the root of your catkin workspace!

2. Navigate to catkin workspace and initialize the `wstool` workspace:
```
cd ~/catkin_ws/src && git clone https://github.com/ivaROS/TrajectoryServoing.git
```

3. Initialize the `wstool` workspace:
```
cd ~/catkin_ws && wstool init src
```

4. Add packages from `TrajectoryServoing.rosinstall` to your catkin workspace:
```
wstool merge -t src src/TrajectoryServoing/TrajectoryServoing.rosinstall
```

5. Download/update all packages managed by wstool:
```
wstool update -t src -j20
rosdep install --from-paths src -i -y
```
Ignore the complaint for any `Cannot locate rosdep definition for [common_rosdeps]`

6. Go to `gazebo_turtlebot_simulator` repository to set up the simulator. Adjust the catkin workspace in __set_up_sim.sh__:

	export CATKIN_WS=/home/XXX/catkin_ws/

Then execute the auto setup script __set_up_sim.sh__ (will be asked for sudo authorization):

	./set_up_sim.sh

7. Build dependencies and supports for `gf_orb_slam2`. Go through `build_dep.sh` in this repository, check if all dependencies are needed, and run this file if some dependencies are not installed. OpenCV 3.4 is optional if 3.3 is installed by ROS kinetic. Then run `build_supports.sh`.

8. Build dependencies for VS+ stereo feature tracking. Go to `stereoFeatureTracking` and run `build_stvo.sh`.

8. Build all packages:

	catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
	catkin build
