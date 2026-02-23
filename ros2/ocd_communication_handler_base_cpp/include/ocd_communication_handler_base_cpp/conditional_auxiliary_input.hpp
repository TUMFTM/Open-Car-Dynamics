// Copyright 2026 Simon Sagmeister
#pragma once
/*

These templated structs provide helper functionality to conditionally add
   auxiliary input setter functions to drivetrain and steering actuator model base classes.

   This is required since an auxiliary input is optional for both drivetrain and steering actuator
models.

   When choosing void as the AuxiliaryInputT type parameter, no auxiliary input is needed.
   The programm would also not compile having a function with the signature func(void).

   For that case we inherit from this templated struct. Via template specialization we provide two
different implementations, for the void type and the non-void type.

*/

namespace auxiliary::conditional_compilation
{

// 1. Declare the struct
template <typename T>
struct AuxiliaryInputHelperCH;
// 2. Define the template overload for the void case
template <>
struct AuxiliaryInputHelperCH<void>
{
  // empty — no function
};
// 3. Define the template overload for the non-void case
template <typename T>
struct AuxiliaryInputHelperCH
{
  virtual T get_auxiliary_input() = 0;
  virtual bool get_new_auxiliary_input_flag() = 0;
  virtual void reset_new_auxiliary_input_flag() = 0;
};
}  // namespace auxiliary::conditional_compilation
