// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <units/current.h>
#include <units/frequency.h>
#include <units/length.h>
#include <units/mass.h>

#include "frc/system/plant/DCMotor.h"

namespace consts::manip {

inline constexpr units::hertz_t BUS_UPDATE_FREQ = 100_Hz;

namespace can_ids {
inline constexpr int ROLLER_MOTOR = 19;
}  // namespace can_ids

namespace current_limits {
inline constexpr units::ampere_t SUPPLY_LIMIT = 40_A;
inline constexpr units::ampere_t STATOR_LIMIT = 40_A;
}  // namespace current_limits

namespace physical {
inline constexpr frc::DCMotor MOTOR = frc::DCMotor::Falcon500FOC(1);
inline constexpr bool INVERT_ROLLER = false;
}  // namespace physical

namespace gains {}  // namespace gains
}  // namespace consts::manip
