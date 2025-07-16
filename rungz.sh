


# colcon build --symlink-install
colcon build --packages-select my_robot_launch
# colcon build --packages-select web_gui_control
# colcon build --packages-select slam_launch
source install/setup.bash


# ros2 launch my_robot_launch robot_body_launch_sim.py  
# ros2 launch my_robot_launch robot_body_launch_sim.py  world:=./src/my_robot_launch/config/cafe.world
ros2 launch my_robot_launch robot_body_launch_sim.py  use_sim_time:=true   world:=./src/my_robot_launch/config/lab.world
# sim_mode:=true
# ros2 launch slam_toolbox online_async_launch.py params_file:=./src/slam_launch/config/mapper_params_online_async.yaml 
# ros2 run rviz2 rviz2 -d ~/myspace/autoJetsonBot/src/my_robot_launch/config/main.rviz --ros-args --param use_sim_time:=true

# 


# rviz2 -d ./src/my_robot_launch/config/main.rviz

# ros2 launch ros_gz_sim ros_gz_spawn_model.launch.py \
#     world:=empty file:=$(ros2 pkg prefix \
#     --share ros_gz_sim_demos)/models/vehicle/model.sdf \
#     entity_name:=my_vehicle \
#     x:=5.0 y:=5.0 z:=0.5 \
#     bridge_name:=ros_gz_bridge \
#     config_file:=gz_conf.xml



