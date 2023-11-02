
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

Start the Gazebo SITL simulation with a drone with video stream

```bash
MicroXRCEAgent udp4 -p 8888 &
cd PX4-Autopilot/
make px4_sitl gazebo-classic_typhoon_h480
```

or with the basic Iris drone

```bash
MicroXRCEAgent udp4 -p 8888 &
cd PX4-Autopilot/
make px4_sitl gazebo
```

### Basic running of a package

Launch container:

```bash
cd px4_simulator_container
./launch_container.sh
```

Set environment variables:

```bash
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/ros_ws/gazebo_plugin/build
export PX4_SITL_WORLD=/ros_ws/gazebo_plugin/model_push.world
```

Change directory to the ros_ws and build it:

```bash
cd /ros_ws
colcon build
```

Alternatively, build only certain packages:

```bash
colcon build --packages-select <name-of-pkg>
```

Source the setup file (in all new terminal sessions):

```bash
source /ros_ws/install/setup.bash
```

#### Running the target_offset package

Do the set-up steps above, then in one terminal start Gazebo:

```bash
cd PX4-Autopilot/
make px4_sitl gazebo-classic_typhoon_h480
```

And in another terminal (remember to source), run the node:

```bash
ros2 run target_offset oc
```

There will be a warning in the simulation regarding the connection, but you can disregard this.

In a third terminal, run:

```bash
ros2 topic echo visual_servo/target_offset
```

to see the offsets published. The offsets are published as a Vector3Stamped with (x_offset, y_offset, proportion), where the offsets are normalized values in the range [-1, 1] in the x and y directions (from the center) and proportion is the proportion of the frame that the bounding box takes up in decimal.

A launch file that starts the oc and video nodes has been added and can be run from the launch folder:

```bash
ros2 launch launch_oc_and_video.py
```

#### Running the visual_servo package

```bash
ros2 run visual_servo offboard
```

Run oc_node and offboard node using a launch file.

```bash
ros2 launch visual_servo vservo_video.launch.py
```
