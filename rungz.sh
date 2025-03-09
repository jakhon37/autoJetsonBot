ros2 launch ros_gz_sim ros_gz_spawn_model.launch.py \
    world:=empty file:=$(ros2 pkg prefix \
    --share ros_gz_sim_demos)/models/vehicle/model.sdf \
    entity_name:=my_vehicle \
    x:=5.0 y:=5.0 z:=0.5 \
    bridge_name:=ros_gz_bridge \
    config_file:=gz_conf.xml
