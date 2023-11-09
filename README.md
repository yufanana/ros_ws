
# ros_ws

ros_ws for px4_simulator_container

This creates a ros_ws folder inside the px4_simulator_container folder.

The visual_servo package was based off the marker_localization_with_controller repo

## Set Up Repository

```bash
cd px4_simulator_containor/
git clone git@github.com:yufanana/ros_ws.git
```

## 1. Launch Container

```bash
cd px4_simulator_container
./launch_container.sh
```

Use tmux to run multiple terminals

```bash
tmux
```

## 2a. Run Gazebo simulation with video stream

Set environment variable to move ball in empty world.

```bash
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/ros_ws/gazebo_plugin/build
```

Set environment variable to load an empty world with a ball.

```bash
export PX4_SITL_WORLD=/ros_ws/gazebo_plugin/model_push.world
```

Start the Gazebo SITL simulation with a drone with video stream

```bash
MicroXRCEAgent udp4 -p 8888 &
cd PX4-Autopilot/
make px4_sitl gazebo-classic_typhoon_h480
```

## 2b. Run Gazebo simulation with Iris drone

Set environment variable to load an empty world with a ball.

```bash
export PX4_SITL_WORLD=/ros_ws/gazebo_plugin/model_push.world
```

Start the Gazebo SITL simulation with a drone with video stream

```bash
MicroXRCEAgent udp4 -p 8888 &
cd PX4-Autopilot/
make px4_sitl gazebo
```

## 3. ros2 launch packages

In one terminal, start Gazebo.

```bash
cd PX4-Autopilot/
make px4_sitl gazebo
```

In another terminal, change directory to the ros_ws and build it:

```bash
cd /ros_ws
colcon build
```

Alternatively, build only certain packages:

```bash
colcon build --packages-select <name-of-pkg>
```

In all new terminal sessions, source the setup file:

```bash
source install/setup.bash
```

Launch the oc_node and offboard_node using launch.py

```bash
ros2 launch visual_servo vservo_video.launch.py
```

## ros2 run the visual_servo package

In another terminal, (remember to source):

```bash
ros2 run visual_servo offboard
```

## ros2 run the target_offset package

In another terminal, (remember to source):

```bash
ros2 run target_offset oc
```

There will be a warning in the simulation regarding the connection, but you can disregard this.

In a third terminal, run:

```bash
ros2 topic echo visual_servo/target_offset
```

to see the offsets published. The offsets are published as a Vector3Stamped with (x_offset, y_offset, proportion), where the offsets are normalized values in the range [-1, 1] in the x and y directions (from the center) and proportion is the proportion of the frame that the bounding box takes up in decimal.

A launch file that starts the oc and video nodes has been added and can be run using:

```bash
ros2 launch target_offset oc_and_video.launch.py
```
