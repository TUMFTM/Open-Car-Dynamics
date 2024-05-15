# External Forces and Moments on the vehicle. If not specified else, values apply to the
# Center of Gravity in the vehicle_frame 
std_msgs/Header header # Pls use vehicle_cg_footprint as frame id if possible
geometry_msgs/Vector3 external_force # N
geometry_msgs/Vector3 external_torque # Nm
tum_msgs/TUMFloat64PerWheel z_height_road_m
tum_msgs/TUMFloat64PerWheel lambda_mue