// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <units/current.h>
#include <units/frequency.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/velocity.h>

#include "frc/system/plant/DCMotor.h"
#include "str/GainTypes.h"
#include "units/dimensionless.h"

namespace consts::elevator {

inline constexpr units::hertz_t BUS_UPDATE_FREQ = 250_Hz;

namespace can_ids {
inline constexpr int FRONT_MOTOR = 15;
inline constexpr int BACK_MOTOR = 16;
}  // namespace can_ids

namespace current_limits {
inline constexpr units::ampere_t SUPPLY_LIMIT = 40_A;
inline constexpr units::ampere_t STATOR_LIMIT = 60_A;
}  // namespace current_limits

namespace physical {
inline constexpr frc::DCMotor MOTOR = frc::DCMotor::Falcon500FOC(2);
inline constexpr bool INVERT_FRONT = true;

inline constexpr units::scalar_t PULLEY_FUDGE_FACTOR = 0.9710598950427783;
inline constexpr units::meter_t PULLEY_DIAM =
    1.7566685458330675_in * PULLEY_FUDGE_FACTOR;
inline constexpr int NUM_OF_STAGES = 3;
inline constexpr units::scalar_t GEARING =
    (3.0 / 1.0) * (44.0 / 16.0) * (32.0 / 14.0);

inline constexpr units::kilogram_t CARRIAGE_MASS = 201.54_lb;

inline constexpr units::meter_t EXTENDED_HEIGHT = 26_in;
}  // namespace physical

namespace gains {

inline constexpr units::meter_t HEIGHT_TOLERANCE = .5_in;

inline const str::gains::linear::VoltLinearGainsHolder ELEVATOR_GAINS{
    ((consts::elevator::physical::MOTOR.freeSpeed /
      consts::elevator::physical::GEARING) /
     1_rad) *
        (consts::elevator::physical::PULLEY_DIAM / 2.0),
    str::gains::linear::meter_volt_ka_unit_t{0},
    str::gains::linear::meter_volt_kv_unit_t{0},
    str::gains::linear::meter_volt_ka_unit_t{0.036699},
    str::gains::linear::meter_volt_kv_unit_t{2.3775},
    0.047698_V,
    str::gains::linear::meter_volt_kp_unit_t{18},
    str::gains::linear::meter_volt_ki_unit_t{0},
    str::gains::linear::meter_volt_kd_unit_t{.1},
};

inline constexpr units::volt_t kG = 0.40627_V;

}  // namespace gains
}  // namespace consts::elevator
