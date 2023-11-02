
# ros_ws

ros_ws for px4_simulator_container

This creates a ros_ws folder inside the px4_simulator_container folder.

## Set Up Repo

```bash
cd px4_simulator_containor/
git clone git@github.com:yufanana/ros_ws.git
```

## Run Gazebo simulation

Launch container

```bash
cd px4_simulator_container
./launch_container.sh
```

Set environment variables

```bash
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/ros_ws/gazebo_plugin/build
export PX4_SITL_WORLD=/ros_ws/gazebo_plugin/model_push.world
```

Start the Gazebo SITL simulation

```bash
MicroXRCEAgent udp4 -p 8888 &
cd PX4-Autopilot/
make px4_sitl gazebo-classic_typhoon_h480
```

### Basic running of a package

Launch container:
```
cd px4_simulator_container
./launch_container.sh
```

Set environment variables:
```
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/ros_ws/gazebo_plugin/build
export PX4_SITL_WORLD=/ros_ws/gazebo_plugin/model_push.world
```

Change directory to the ros_ws and build it:
```
cd /ros_ws
colcon build
```

Alternatively, build only certain packages:
```
colcon build --packages-select <name-of-pkg>
```

Source the setup file (in all new terminal sessions):
```
source /ros_ws/install/setup.bash
```

#### Running the target_offset package
Do the set-up steps above, then in one terminal start Gazebo:
```
cd PX4-Autopilot/
make px4_sitl gazebo-classic_typhoon_h480
```

And in another terminal (remember to source), run the node:
```
ros2 run target_offset oc
```
There will be a warning in the simulation regarding the connection, but you can disregard this.

In a third terminal, run:
```
ros2 topic echo /offsets
```
to see the offsets published. The offsets are published as a Vector3Stamped with (x_offset, y_offset, proportion), where the offsets are normalized values in the range [-1, 1] in the x and y directions (from the center) and proportion is the proportion of the frame that the bounding box takes up in decimal.

A launch file that starts the oc and video nodes has been added and can be run from the launch folder:
```
ros2 launch launch_oc_and_video.py
```