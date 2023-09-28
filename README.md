# ros_ws
ros_ws for px4_simulator_container

This creates a ros_ws folder inside the px4_simulator_container folder.


### Set Up Repo
```
cd px4_simulator_containor/
git clone git@github.com:yufanana/ros_ws.git
```

### Run Gazebo simulation

```
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/ros_ws/gazebo_plugin/build
export PX4_SITL_WORLD=/ros_ws/gazebo_plugin/model_push.world
make px4_sitl gazebo-classic_typhoon_h480
```
