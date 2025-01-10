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
#include "str/Units.h"
#include "units/dimensionless.h"

namespace consts::elevator {

inline constexpr units::hertz_t BUS_UPDATE_FREQ = 250_Hz;

namespace can_ids {
inline constexpr int LEFT_MOTOR = 15;
inline constexpr int RIGHT_MOTOR = 16;
}  // namespace can_ids

namespace current_limits {
inline constexpr units::ampere_t SUPPLY_LIMIT = 40_A;
inline constexpr units::ampere_t STATOR_LIMIT = 60_A;
}  // namespace current_limits

namespace physical {
inline constexpr frc::DCMotor MOTOR = frc::DCMotor::Falcon500(2);
inline constexpr bool INVERT_LEFT = false;
inline constexpr bool INVERT_RIGHT = true;

inline constexpr units::meter_t PULLEY_DIAM = 1.75_in;
inline constexpr int NUM_OF_STAGES = 2;
inline constexpr units::scalar_t GEARING = (10) * (1.0 / NUM_OF_STAGES);

inline constexpr units::kilogram_t CARRIAGE_MASS = 20_lb;

inline constexpr units::meter_t EXTENDED_HEIGHT = 6_ft;
}  // namespace physical

namespace gains {
struct holder {
  units::meters_per_second_t motionMagicCruiseVel;
  str::gains::linear::meter_volt_ka_unit_t motionMagicExpoKa;
  str::gains::linear::meter_volt_kv_unit_t motionMagicExpoKv;
  str::gains::linear::meter_amp_ka_unit_t kA;
  str::gains::linear::meter_amp_kv_unit_t kV;
  units::ampere_t kS;
  str::gains::linear::meter_amp_kp_unit_t kP;
  str::gains::linear::meter_amp_ki_unit_t kI;
  str::gains::linear::meter_amp_kd_unit_t kD;

  holder& operator=(const holder& other) = default;
  holder(const holder& other)
      : motionMagicCruiseVel{other.motionMagicCruiseVel},
        motionMagicExpoKa{other.motionMagicExpoKa},
        motionMagicExpoKv{other.motionMagicExpoKv},
        kA{other.kA},
        kV{other.kV},
        kS{other.kS},
        kP{other.kP},
        kI{other.kI},
        kD{other.kD} {}
  holder(units::meters_per_second_t mmCv,
         str::gains::linear::meter_volt_ka_unit_t mmKa,
         str::gains::linear::meter_volt_kv_unit_t mmKv,
         str::gains::linear::meter_amp_ka_unit_t ka,
         str::gains::linear::meter_amp_kv_unit_t kv, units::ampere_t ks,
         str::gains::linear::meter_amp_kp_unit_t kp,
         str::gains::linear::meter_amp_ki_unit_t ki,
         str::gains::linear::meter_amp_kd_unit_t kd)
      : motionMagicCruiseVel{mmCv},
        motionMagicExpoKa{mmKa},
        motionMagicExpoKv{mmKv},
        kA{ka},
        kV{kv},
        kS{ks},
        kP{kp},
        kI{ki},
        kD{kd} {}

  bool operator==(const holder& rhs) const {
    return units::essentiallyEqual(motionMagicCruiseVel,
                                   rhs.motionMagicCruiseVel, 1e-6),
           units::essentiallyEqual(motionMagicExpoKa, rhs.motionMagicExpoKa,
                                   1e-6),
           units::essentiallyEqual(motionMagicExpoKv, rhs.motionMagicExpoKv,
                                   1e-6),
           units::essentiallyEqual(kA, rhs.kA, 1e-6) &&
               units::essentiallyEqual(kV, rhs.kV, 1e-6) &&
               units::essentiallyEqual(kS, rhs.kS, 1e-6) &&
               units::essentiallyEqual(kP, rhs.kP, 1e-6) &&
               units::essentiallyEqual(kI, rhs.kI, 1e-6) &&
               units::essentiallyEqual(kD, rhs.kD, 1e-6);
  }
  bool operator!=(const holder& rhs) const { return !operator==(rhs); }
};

inline const holder ELEVATOR_GAINS{
    (physical::MOTOR.freeSpeed / physical::GEARING) / (1_rad) *
        (physical::PULLEY_DIAM / 2),
    str::gains::linear::meter_volt_ka_unit_t{0},
    str::gains::linear::meter_volt_kv_unit_t{0},
    str::gains::linear::meter_amp_ka_unit_t{0},
    str::gains::linear::meter_amp_kv_unit_t{0},
    0_A,
    str::gains::linear::meter_amp_kp_unit_t{250},
    str::gains::linear::meter_amp_ki_unit_t{0},
    str::gains::linear::meter_amp_kd_unit_t{0},
};

}  // namespace gains
}  // namespace consts::elevator
