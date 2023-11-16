# Connection between Carla Ros Bridge <-> Raspberry
Our intention was to create a node that would fetch data (Speed, direction flag, accelerometer coordinates) from the Carla Simulator and then pass it on to our controllers/nodes on Raspberry Pi.

# How to install it?

## What do we need for the installation?
To be able to install this, we need:
- Ubuntu Server + ROS1
- Carla Simulator
- Carla-Ros-Bridge
- Catkin
- Python

## 
Now, navigate to the folder where you want to have your project and download our repository:

```
git clone https://github.com/Kakub02/PBL_DriverInTheLoop.git
```
Now we need to enter catkin_ws/ and build our project:
```
cd path/to/catkin_ws/
catkin_make
```

You may encounter a dependency issue, here is one thing that might help:
```
rosdep update
rosdep install --from-paths src --ignore-src -r
```

Now you need to source the setup.bash file in both the 'ROS' and 'devel' folders (I recommend being in the 'src' folder when you do this):
```
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

# It's time to start the machine.
At the very beginning, enter the folder with the Carla Simulator, source to Ros and devel and run CarlaUE4.sh

```
source path_to_/opt/ros/noetic/setup.bash
source path_to_/devel/setup.bash

cd path_to_carla

./CarlaUE4.sh
```

Open one terminal and start roscore:
```
roscore
```
Now, in other terminal:
Go to the ROS-bridge package, source it, and start the ROS-bridge:
```
roslaunch carla_ros_bridge carla_ros_bridge.launch
```

To check if it's working, you can use the command provided below and check the list of topics published by Carla-Ros-Bridge:
```
rostopic list
```
To chceck topics you can use:
```
rostopic echo /our/topic
```

# Start the node on the PC.
Now, go to our Catkin workspace, navigate to data_collector/scripts, source the setup.bash as before, and then start our node:
```
cd path_to_catkin/catkin_ws/data_collector_scripts
source /opt/ros/noetic/setup.bash
source path_to/devel/setup.bash

!important
rosrun <package-name> <script-name>
rosrun data_collector main.py
```
Here, our node is already running and will transmit data to the Raspberry Pi in the future (we don't have subscribers yet). To check if it's working, use:
```
rostopic list
```

If you see topics like:
```
/our_msg/speed
/our_msg/roll
/our_msg/pitch
/our_msg/reverse
```
It means our node is working.

