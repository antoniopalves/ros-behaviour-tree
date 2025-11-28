Build
´´´
cd ~/ros2_ws
colcon build --packages-select trsa_bt_plugins
source install/setup.bash
´´´

Run 
Terminal 1 
´´´
ros2 launch nav2_bringup tb3_simulation_launch.py \
  headless:=False \
  params_file:=/home/antoniopalves/ros2_ws/src/ros-behaviour-tree/trsa_bt_plugins/config/nav2_params.yaml \
  world:=/home/antoniopalves/ros2_ws/src/ros-behaviour-tree/trsa_bt_plugins/worlds/trsa_tb3_cone.world
´´´ 
Terminal 2
´´´
ros2 run trsa_bt_plugins cone_detector.py
´´´
