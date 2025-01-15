// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <units/current.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/frequency.h>

#include "frc/system/plant/DCMotor.h"
#include "units/angle.h"
#include "units/dimensionless.h"
#include "units/moment_of_inertia.h"
#include "str/GainTypes.h"

namespace consts::arm {

inline constexpr units::hertz_t BUS_UPDATE_FREQ = 100_Hz;

namespace can_ids {
inline constexpr int PIVOT_MOTOR = 17;
}  // namespace can_ids

namespace current_limits {
inline constexpr units::ampere_t SUPPLY_LIMIT = 40_A;
inline constexpr units::ampere_t STATOR_LIMIT = 10_A;
}  // namespace current_limits

namespace physical {
inline constexpr frc::DCMotor MOTOR = frc::DCMotor::Falcon500FOC(1);
inline constexpr bool INVERT_PIVOT = false;

inline constexpr units::scalar_t GEARING = 60;

inline constexpr units::kilogram_t MASS = 10_lb;
inline constexpr units::kilogram_square_meter_t MOI = 0.12009477_kg_sq_m;

inline constexpr units::meter_t ARM_LENGTH = 4.37480954214_in;

inline constexpr units::radian_t MIN_ANGLE = -359_deg;
inline constexpr units::radian_t MAX_ANGLE = 360_deg;
}  // namespace physical

namespace gains {
inline const str::gains::radial::RadialGainsHolder PIVOT_GAINS{
    consts::arm::physical::MOTOR.freeSpeed / consts::arm::physical::GEARING,
    str::gains::radial::turn_volt_ka_unit_t{0},
    str::gains::radial::turn_volt_kv_unit_t{0},
    str::gains::radial::turn_amp_ka_unit_t{0},
    str::gains::radial::turn_amp_kv_unit_t{0},
    0_A,
    str::gains::radial::turn_amp_kp_unit_t{0},
    str::gains::radial::turn_amp_ki_unit_t{0},
    str::gains::radial::turn_amp_kd_unit_t{0},
};
}  // namespace gains
}  // namespace consts::arm
