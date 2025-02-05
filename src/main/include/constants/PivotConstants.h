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

namespace consts::pivot {

inline constexpr units::hertz_t BUS_UPDATE_FREQ = 100_Hz;

namespace can_ids {
inline constexpr int PIVOT_MOTOR = 17;
inline constexpr int PIVOT_ENC = 18;
}  // namespace can_ids

namespace current_limits {
inline constexpr units::ampere_t SUPPLY_LIMIT = 40_A;
inline constexpr units::ampere_t STATOR_LIMIT = 40_A;
}  // namespace current_limits

namespace physical {
inline constexpr frc::DCMotor MOTOR = frc::DCMotor::Falcon500FOC(1);
inline constexpr bool INVERT_PIVOT = false;

inline constexpr units::scalar_t GEARING = 21.333;

inline constexpr units::kilogram_t MASS = 10_lb;
inline constexpr units::kilogram_square_meter_t MOI = 0.12009477_kg_sq_m;

inline constexpr units::meter_t ARM_LENGTH = 4.37480954214_in;

inline constexpr units::radian_t MIN_ANGLE = -360_deg;
inline constexpr units::radian_t MAX_ANGLE = 360_deg;

inline constexpr units::turn_t ENCODER_OFFSET = .32_tr;

inline constexpr units::radian_t CLEAR_OF_FUNNEL_ANGLE = 70_deg;
}  // namespace physical

namespace gains {

inline constexpr units::radian_t ANGLE_TOLERANCE = 1_deg;

inline const str::gains::radial::VoltRadialGainsHolder PIVOT_GAINS{
    consts::pivot::physical::MOTOR.freeSpeed / consts::pivot::physical::GEARING,
    str::gains::radial::turn_volt_ka_unit_t{0.073766},
    str::gains::radial::turn_volt_kv_unit_t{2.8397},
    str::gains::radial::turn_volt_ka_unit_t{0.073766},
    str::gains::radial::turn_volt_kv_unit_t{2.8397},
    0.11888_V,
    str::gains::radial::turn_volt_kp_unit_t{22.604},
    str::gains::radial::turn_volt_ki_unit_t{0},
    str::gains::radial::turn_volt_kd_unit_t{0.61875},
};

inline constexpr units::volt_t kG = 1.5_V;

}  // namespace gains
}  // namespace consts::pivot
