// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <units/current.h>
#include <units/frequency.h>
#include <units/length.h>
#include <units/mass.h>

#include "frc/system/plant/DCMotor.h"
#include "str/GainTypes.h"
#include "units/angle.h"
#include "units/dimensionless.h"
#include "units/moment_of_inertia.h"

namespace consts::algae {

inline constexpr units::hertz_t BUS_UPDATE_FREQ = 100_Hz;

namespace can_ids {
inline constexpr int ALGAE_PIVOT_MOTOR = 20;
inline constexpr int ALGAE_ROLLER_MOTOR = 21;
}  // namespace can_ids

namespace current_limits {
inline constexpr units::ampere_t SUPPLY_LIMIT = 20_A;
inline constexpr units::ampere_t STATOR_LIMIT = 20_A;
inline constexpr units::ampere_t GRABBED_ALGAE = 30_A;
}  // namespace current_limits

namespace physical {
inline constexpr frc::DCMotor PIVOT_MOTOR = frc::DCMotor::Falcon500FOC(1);
inline constexpr frc::DCMotor ROLLER_MOTOR = frc::DCMotor::Falcon500FOC(1);

inline constexpr bool INVERT_PIVOT = true;
inline constexpr bool INVERT_ROLLER = false;

inline constexpr units::scalar_t PIVOT_GEARING = 13.909;
inline constexpr units::scalar_t ROLLER_GEARING = 20.2500;
inline constexpr units::kilogram_square_meter_t ROLLER_MOI =
    0.000107848_kg_sq_m;

inline constexpr units::kilogram_t MASS = 1.3_lb;
inline constexpr units::kilogram_square_meter_t MOI = 0.009044167_kg_sq_m;

inline constexpr units::meter_t ARM_LENGTH = 13.219105_in;

inline constexpr units::radian_t MIN_ANGLE = -90_deg;
inline constexpr units::radian_t MAX_ANGLE = 100_deg;

inline constexpr units::radian_t ALGAE_INTAKE_ANGLE = -0_deg;
inline constexpr units::radian_t ALGAE_STOW_ANGLE = -80_deg;
inline constexpr units::radian_t ALGAE_HOLD_ANGLE = -25_deg;
}  // namespace physical

namespace gains {

inline constexpr units::radian_t ANGLE_TOLERANCE = 1_deg;

inline const str::gains::radial::VoltRadialGainsHolder ALGAE_PIVOT_GAINS{
    consts::algae::physical::PIVOT_MOTOR.freeSpeed /
        consts::algae::physical::PIVOT_GEARING,
    str::gains::radial::turn_volt_ka_unit_t{5},
    str::gains::radial::turn_volt_kv_unit_t{10},
    str::gains::radial::turn_volt_ka_unit_t{0},
    str::gains::radial::turn_volt_kv_unit_t{0},
    0.0_V,
    str::gains::radial::turn_volt_kp_unit_t{25},
    str::gains::radial::turn_volt_ki_unit_t{0},
    str::gains::radial::turn_volt_kd_unit_t{0},
};

inline constexpr units::volt_t kG = .35_V;

}  // namespace gains
}  // namespace consts::algae
