// Copyright 2026 Simon Sagmeister
#pragma once
namespace auxiliary::conditional_compilation
{

/*

These templated structs provide helper functionality to conditionally add
   auxiliary input setter functions to accomomodate drivetrain and steering actuator.

   This is required since an auxiliary input is optional for both drivetrain and steering actuator
models.

   When choosing void as the AuxiliaryInputT type parameter, no auxiliary input is needed.
   The programm would also not compile having a function with the signature func(void).

   For that case we inherit from this templated struct. Via template specialization we provide two
different implementations, for the void type and the non-void type.

*/

// 1. Declare the struct
// Hereby inherit from the VehicleModelBase to be act as a base class for VehicleModel
template <typename DT_MODEL_T, typename SA_MODEL_T, typename DT_AUX_INPUT_T>
struct DtHelperVehicleModel;
// 2. Define the template overload for the void case
template <typename DT_MODEL_T, typename SA_MODEL_T>
struct DtHelperVehicleModel<DT_MODEL_T, SA_MODEL_T, void>
: public tam::ocd::interfaces::VehicleModelBase<
    typename DT_MODEL_T::DriverInputType, typename DT_MODEL_T::FeedbackType,
    typename SA_MODEL_T::DriverInputType, typename SA_MODEL_T::FeedbackType,
    typename DT_MODEL_T::AuxiliaryInputType, typename SA_MODEL_T::AuxiliaryInputType>
{
protected:
  // Create the instance of the drivetran model here to have access in the derived class
  DT_MODEL_T drivetrain_model_;
};
// 3. Define the template overload for the non-void case
template <typename DT_MODEL_T, typename SA_MODEL_T, typename DT_AUX_INPUT_T>
struct DtHelperVehicleModel
: public tam::ocd::interfaces::VehicleModelBase<
    typename DT_MODEL_T::DriverInputType, typename DT_MODEL_T::FeedbackType,
    typename SA_MODEL_T::DriverInputType, typename SA_MODEL_T::FeedbackType,
    typename DT_MODEL_T::AuxiliaryInputType, typename SA_MODEL_T::AuxiliaryInputType>
{
  // Create the instance of the drivetran model here to have access in the derived class
protected:
  DT_MODEL_T drivetrain_model_;
  // Implement the auxiliary input setter function
public:
  void set_auxiliary_input_drivetrain(const DT_AUX_INPUT_T & value) override
  {
    drivetrain_model_.set_auxiliary_input(value);
  }
};

// Now repeat the same for the steering actuator model

// 1. Declare the struct
// This time inherit from the previous struct to have both models available and to avoi
// inheriting from VehicleModelBase multiple times. Otherwise non implemented
// virtual functions would cause issues and override each other.
template <typename DT_MODEL_T, typename SA_MODEL_T, typename SA_AUX_INPUT_T>
struct SaHelperVehicleModel;
// 2. Define the template overload for the void case
template <typename DT_MODEL_T, typename SA_MODEL_T>
struct SaHelperVehicleModel<DT_MODEL_T, SA_MODEL_T, void>
: public DtHelperVehicleModel<DT_MODEL_T, SA_MODEL_T, typename DT_MODEL_T::AuxiliaryInputType>
{
protected:
  // Create the instance of the steering actuator model here to have access in the derived class
  SA_MODEL_T steering_actuator_model_;
  // Also bring in the drivetrain model from the base struct
  using DtHelperVehicleModel<
    DT_MODEL_T, SA_MODEL_T, typename DT_MODEL_T::AuxiliaryInputType>::drivetrain_model_;
};
// 3. Define the template overload for the non-void case
template <typename DT_MODEL_T, typename SA_MODEL_T, typename SA_AUX_INPUT_T>
struct SaHelperVehicleModel
: public DtHelperVehicleModel<DT_MODEL_T, SA_MODEL_T, typename DT_MODEL_T::AuxiliaryInputType>
{
protected:
  // Create the instance of the steering actuator model here to have access in the derived class
  SA_MODEL_T steering_actuator_model_;
  // Also bring in the drivetrain model from the base struct
  using DtHelperVehicleModel<
    DT_MODEL_T, SA_MODEL_T, typename DT_MODEL_T::AuxiliaryInputType>::drivetrain_model_;
  // Implement the auxiliary input setter function
public:
  void set_auxiliary_input_steering_actuator(const SA_AUX_INPUT_T & value) override
  {
    steering_actuator_model_.set_auxiliary_input(value);
  }
};
}  // namespace auxiliary::conditional_compilation
