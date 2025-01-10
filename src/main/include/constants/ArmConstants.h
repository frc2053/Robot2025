// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <units/current.h>
#include <units/length.h>
#include "units/dimensionless.h"
#include <units/mass.h>

namespace consts::arm {
namespace can_ids {
inline constexpr int PIVOT = 17;
}  // namespace can_ids

namespace current_limits {
inline constexpr units::ampere_t SUPPLY_LIMIT = 40_A;
inline constexpr units::ampere_t STATOR_LIMIT = 60_A;
}  // namespace current_limits

namespace physical {
inline constexpr units::scalar_t GEARING = 60;

inline constexpr units::kilogram_t MASS = 20_lb;

inline constexpr units::meter_t ARM_LENGTH = 18_in;
inline constexpr units::meter_t END_EFFECTOR_LENGTH = 12_in;
}  // namespace physical

namespace gains {}
}  // namespace consts::arm