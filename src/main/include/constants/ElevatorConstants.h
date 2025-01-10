// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <units/current.h>
#include <units/length.h>
#include <units/mass.h>

#include "units/dimensionless.h"

namespace consts::elevator {
namespace can_ids {
inline constexpr int LEFT_MOTOR = 15;
inline constexpr int RIGHT_MOTOR = 16;
}  // namespace can_ids

namespace current_limits {
inline constexpr units::ampere_t SUPPLY_LIMIT = 40_A;
inline constexpr units::ampere_t STATOR_LIMIT = 60_A;
}  // namespace current_limits

namespace physical {
inline constexpr units::meter_t PULLEY_DIAM = 1.75_in;
inline constexpr units::scalar_t GEARING = 10;

inline constexpr units::kilogram_t CARRIAGE_MASS = 20_lb;

inline constexpr units::meter_t EXTENDED_HEIGHT = 6_ft;
}  // namespace physical

namespace gains {}  // namespace gains
}  // namespace consts::elevator
