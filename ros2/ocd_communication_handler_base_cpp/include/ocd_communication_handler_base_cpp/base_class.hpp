// Copyright 2026 Simon Sagmeister
#pragma once

#include <chrono>
#include <memory>

#include "ocd_communication_handler_base_cpp/conditional_auxiliary_input.hpp"
#include "ocd_communication_handler_base_cpp/conditional_feedback.hpp"
#include "param_management_cpp/base.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tsl_logger_cpp/base.hpp"
namespace tam::ocd::interfaces
{
template <typename DriverInputT, typename FeedbackT, typename AuxiliaryInputT = void>
class CommunicationHandlerBase
// Inherit conditional auxiliary input and feedback helpers
: public auxiliary::conditional_compilation::AuxiliaryInputHelperCH<AuxiliaryInputT>,
  public auxiliary::conditional_compilation::FeedbackHelperCH<FeedbackT>
{
public:
  using DriverInputType = DriverInputT;
  using FeedbackType = FeedbackT;
  using AuxiliaryInputType = AuxiliaryInputT;

  // Virtual destructor
  virtual ~CommunicationHandlerBase() = default;

  // Input/feedback handling
  virtual DriverInputType get_driver_input() = 0;
  // Conditionally Inherit from FeedbackHelperCH
  // virtual void publish_feedback(FeedbackType feedback, rclcpp::Time timeSstamp) = 0;

  // Delay handling
  virtual void update_delay_timer(std::chrono::duration<double> integration_step_s) = 0;
  virtual void reset_input_delay() = 0;

  // Flags for efficient input handling
  virtual bool get_new_input_flag() = 0;
  virtual void reset_new_input_flag() = 0;

  /* These functions are only available if AuxiliaryInputType is not void and are inherited from the
  helper struct
  virtual AuxiliaryInputType get_auxiliary_input() = 0;
  virtual bool get_new_auxiliary_input_flag() = 0;
  virtual void reset_new_auxiliary_input_flag() = 0;
  */

  // Parameter Management and Logging
  virtual tam::pmg::MgmtInterface::SharedPtr get_param_manager() = 0;
  virtual tam::tsl::LoggerAccessInterface::SharedPtr get_logger() = 0;
};
}  // namespace tam::ocd::interfaces
