# Aggregated external influences acting on the vehicle. Unless stated otherwise,
# values apply to the Center of Gravity expressed in the vehicle frame.
std_msgs/Header header  # Pls use vehicle_cg_footprint as frame id if possible
geometry_msgs/Vector3 external_force   # N
geometry_msgs/Vector3 external_torque  # Nm
tum_msgs/TUMFloat64PerWheel lambda_mue  # Per-wheel friction scale (1.0 == nominal)
tum_msgs/TUMFloat64PerWheel z_height_road_m
