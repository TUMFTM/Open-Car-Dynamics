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
struct AuxiliaryInputHelperSA;
// 2. Define the template overload for the void case
template <>
struct AuxiliaryInputHelperSA<void>
{
  // empty — no function
};
// 3. Define the template overload for the non-void case
template <typename T>
struct AuxiliaryInputHelperSA
{
  virtual void set_auxiliary_input(const T & value) = 0;
};
}  // namespace auxiliary::conditional_compilation
