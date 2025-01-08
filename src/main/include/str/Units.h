// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <units/base.h>
#include <units/math.h>

namespace units {

template <typename T>
bool essentiallyEqual(T a, T b, units::scalar_t epsilon) {
  return units::math::abs(a - b) <=
         ((units::math::abs(a) > units::math::abs(b) ? units::math::abs(b)
                                                     : units::math::abs(a)) *
          epsilon);
}
}  // namespace units
