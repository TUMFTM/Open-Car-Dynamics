// Copyright 2023 Simon Hoffmann
#include "vehicle_model_nodes/vehicle_model_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
VehicleModelNode::VehicleModelNode(std::unique_ptr<tam::interfaces::VehicleModelBase> && dyn_model)
: Node(std::string(node_name_), std::string(namespace_)), veh_model_(std::move(dyn_model))
{
  callback_handle_ =
    tam::ros::connect_param_manager_to_ros_cb(this, veh_model_->get_param_manager());
  tam::ros::declare_ros_params_from_param_manager(this, veh_model_->get_param_manager());

  // model update timer
  timer_period_s_ =
    std::chrono::duration<double>(this->get_parameter("integration_step_size_s").as_double());
  model_update_timer_ = this->create_wall_timer(
    timer_period_s_, std::bind(&VehicleModelNode::model_update_callback, this));

  output_timer_ =
    this->create_wall_timer(10ms, std::bind(&VehicleModelNode::output_callback, this));

  sub_ctrl_cmd_ =
    this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      std::string(topic_sub_ctrl_cmd_), 1,
      std::bind(&VehicleModelNode::ctrl_cmd_callback, this, _1));
  sub_external_influences_ = this->create_subscription<tum_msgs::msg::TUMExternalVehicleInfluences>(
    std::string(topic_sub_external_influences_), 1,
    std::bind(&VehicleModelNode::external_influences_callback, this, _1));

  odometry_pub_ =
    this->create_publisher<nav_msgs::msg::Odometry>(std::string(topic_pub_odometry_), 1);
  accel_pub_ = this->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
    std::string(topic_pub_accel_), 1);
  wheel_speed_pub_ = this->create_publisher<tum_msgs::msg::TUMFloat64PerWheelStamped>(
    std::string(topic_pub_wheelspeeds_), 1);
  steering_angle_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
    std::string(topic_pub_steering_), 1);

  // set initial state
  model_input_.steering_angle_rad = 0.0;
  model_input_.ax_target = 0.0;

  external_input_.external_force_N = tam::types::common::Vector3D<double>(0.0, 0.0, 0.0);
  external_input_.external_torque_Nm = tam::types::common::Vector3D<double>(0.0, 0.0, 0.0);
  external_input_.lambda_mue = tam::types::common::DataPerWheel<double>(1.0, 1.0, 1.0, 1.0);
}
void VehicleModelNode::model_update_callback()
{
  auto t_start = std::chrono::high_resolution_clock::now().time_since_epoch();
  if (has_new_input) update_model_inputs();

  if (has_new_external_influence) update_external_influences();

  veh_model_->step();
  if (initial_cycle_) {
    initial_cycle_ = false;
    return;
  }
  auto t_stop = std::chrono::high_resolution_clock::now().time_since_epoch();
}
void VehicleModelNode::output_callback()
{
  if (initial_cycle_) return;
  tam::types::VehicleModelOutput output = veh_model_->get_output();
  nav_msgs::msg::Odometry odom = tam::helpers::type_conversion::toMsg(output);
  rclcpp::Time stamp = get_clock()->now();

  odom.header.stamp = stamp;
  odom.header.frame_id = "local_cartesian";
  odometry_pub_->publish(odom);

  tum_msgs::msg::TUMFloat64PerWheelStamped wheel_speed;
  wheel_speed.data =
    tam::helpers::type_conversion::toMsg(output.wheel_speed_radps);
  wheel_speed_pub_->publish(wheel_speed);

  geometry_msgs::msg::AccelWithCovarianceStamped accel;
  accel.accel.accel.linear = tam::helpers::type_conversion::toMsg(output.acceleration_mps2);
  accel.accel.accel.angular =
    tam::helpers::type_conversion::toMsg(output.angular_accelaration_radps2);
  accel.header.stamp = stamp;
  accel_pub_->publish(accel);

  autoware_auto_vehicle_msgs::msg::SteeringReport steering_report;
  steering_report.steering_tire_angle = output.steering_angle;
  steering_angle_pub_->publish(steering_report);

  // Publish the debug outputs
  debug_pub_.publish_debug_outputs(veh_model_->get_debug_out().get());
}
void VehicleModelNode::update_model_inputs()
{
  veh_model_->set_driver_input(model_input_);
  has_new_input = false;
}
void VehicleModelNode::update_external_influences()
{
  veh_model_->set_external_influences(external_input_);
  has_new_external_influence = false;
}
void VehicleModelNode::ctrl_cmd_callback(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
{
  model_input_.steering_angle_rad = msg->lateral.steering_tire_angle;
  model_input_.ax_target = msg->longitudinal.acceleration;
  has_new_input = true;
}
void VehicleModelNode::external_influences_callback(
  const tum_msgs::msg::TUMExternalVehicleInfluences::SharedPtr msg)
{
  has_new_external_influence = true;
  external_input_ = tam::helpers::type_conversion::external_influences_type_from_msg(*msg);
}
void VehicleModelNode::reset() { veh_model_->reset(); }
