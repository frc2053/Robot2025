// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <units/current.h>
#include <units/frequency.h>
#include <units/length.h>
#include <units/mass.h>

#include "frc/system/plant/DCMotor.h"
#include "units/moment_of_inertia.h"
#include "units/time.h"
#include "units/voltage.h"

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
inline constexpr units::dimensionless::scalar_t CORAL_REDUCTION = 36.0 / 30.0;
inline constexpr units::moment_of_inertia::kilogram_square_meter_t MOI =
    0.0001_kg_sq_m;
}  // namespace physical

namespace gains {
inline constexpr units::volt_t POOP_VOLTS = 6_V;
inline constexpr units::volt_t SUCK_VOLTS = 10_V;

inline constexpr units::ampere_t GOT_GAME_PIECE_CURRENT = 20_A;
inline constexpr units::ampere_t DROPPED_GAME_PIECE_CURRENT = -20_A;
inline constexpr units::second_t CORAL_DEBOUNCE_TIME = 40_ms;
}  // namespace gains
}  // namespace consts::manip
