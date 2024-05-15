docker run \
-d \
--init \
--rm \
--network=host \
--env ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
--env RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION \
--name vehicle_model_double_track_cpp \
-v  ./config:/dev_ws/config open_car_dynamics:local \
bash -c "source /dev_ws/install/setup.bash && ros2 run vehicle_model_nodes vehicle_model_double_track_cpp_node --ros-args --params-file /dev_ws/config/example_config.yml"
