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

namespace consts::climber {

inline constexpr units::hertz_t BUS_UPDATE_FREQ = 100_Hz;

namespace can_ids {
inline constexpr int CLIMBER_MOTOR = 20;
}  // namespace can_ids

namespace current_limits {
inline constexpr units::ampere_t SUPPLY_LIMIT = 40_A;
inline constexpr units::ampere_t STATOR_LIMIT = 100_A;
}  // namespace current_limits

namespace physical {
inline constexpr frc::DCMotor CLIMBER_MOTOR = frc::DCMotor::Falcon500FOC(1);

inline constexpr bool INVERT_CLIMBER = false;

inline constexpr units::scalar_t GEARING =
    (3 / 1) * (3 / 1) * (3 / 1) * (3 / 1) * (32 / 10);

inline constexpr units::kilogram_t MASS = 4.5930353_lb;
inline constexpr units::kilogram_square_meter_t MOI = 0.015957496_kg_sq_m;

inline constexpr units::meter_t ARM_LENGTH = 3.9944802402_in;

inline constexpr units::radian_t MIN_ANGLE = -90_deg;
inline constexpr units::radian_t MAX_ANGLE = 180_deg;

inline constexpr units::radian_t CLIMB_OUT_ANGLE = .275_tr;
inline constexpr units::radian_t CLIMB_STOW_ANGLE = 0.09_tr;
}  // namespace physical

namespace gains {

inline constexpr units::radian_t ANGLE_TOLERANCE = 1_deg;

inline const str::gains::radial::VoltRadialGainsHolder CLIMBER_GAINS{
    consts::climber::physical::CLIMBER_MOTOR.freeSpeed /
        consts::climber::physical::GEARING,
    str::gains::radial::turn_volt_ka_unit_t{0},
    str::gains::radial::turn_volt_kv_unit_t{0},
    str::gains::radial::turn_volt_ka_unit_t{0},
    str::gains::radial::turn_volt_kv_unit_t{0},
    0.0_V,
    str::gains::radial::turn_volt_kp_unit_t{50},
    str::gains::radial::turn_volt_ki_unit_t{0},
    str::gains::radial::turn_volt_kd_unit_t{0},
};

inline constexpr units::volt_t kG = 0_V;

}  // namespace gains
}  // namespace consts::climber
