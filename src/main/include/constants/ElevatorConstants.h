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
inline constexpr int LEFT_MOTOR = 15;
inline constexpr int RIGHT_MOTOR = 16;
inline constexpr int OUTPUT_ENC = 17;
}  // namespace can_ids

namespace current_limits {
inline constexpr units::ampere_t SUPPLY_LIMIT = 40_A;
inline constexpr units::ampere_t STATOR_LIMIT = 60_A;
}  // namespace current_limits

namespace physical {
inline constexpr frc::DCMotor MOTOR = frc::DCMotor::Falcon500FOC(2);
inline constexpr bool INVERT_LEFT = false;
inline constexpr bool INVERT_RIGHT = true;

inline constexpr units::meter_t PULLEY_DIAM = 1.75_in;
inline constexpr int NUM_OF_STAGES = 3;
inline constexpr units::scalar_t GEARING = 20;

inline constexpr units::kilogram_t CARRIAGE_MASS = 201.54_lb;

inline constexpr units::meter_t EXTENDED_HEIGHT = 26_in;

inline constexpr units::turn_t ENC_OFFSET = .32_tr;
}  // namespace physical

namespace gains {

inline constexpr units::meter_t HEIGHT_TOLERANCE = .25_in;

inline const str::gains::radial::VoltRadialGainsHolder ELEVATOR_GAINS{
    (physical::MOTOR.freeSpeed / physical::GEARING),
    str::gains::radial::turn_volt_ka_unit_t{0.024803},
    str::gains::radial::turn_volt_kv_unit_t{2.3032},
    str::gains::radial::turn_volt_ka_unit_t{0.024803},
    str::gains::radial::turn_volt_kv_unit_t{2.3046},
    0.057849_V,
    str::gains::radial::turn_volt_kp_unit_t{21.058},
    str::gains::radial::turn_volt_ki_unit_t{0},
    str::gains::radial::turn_volt_kd_unit_t{0.14183},
};

inline constexpr units::volt_t kG = 1.1181_V;

}  // namespace gains
}  // namespace consts::elevator
