// Copyright 2023 Simon Hoffmann
#pragma once

#include "tum_types_cpp/common.hpp"
namespace tam::helpers::types
{
inline tam::types::common::Vector3D<double> to_vector3d(
  const tam::types::common::EulerYPR & euler_ypr)
{
  return {euler_ypr.roll, euler_ypr.pitch, euler_ypr.yaw};
}
inline tam::types::common::EulerYPR to_euler_ypr(
  const tam::types::common::Vector3D<double> & vector3d_rpy)
{
  return tam::types::common::EulerYPR(vector3d_rpy.z, vector3d_rpy.y, vector3d_rpy.x);
}
}  // namespace tam::helpers::types
