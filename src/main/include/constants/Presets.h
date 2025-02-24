// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <units/angle.h>
#include <units/length.h>

namespace presets {
namespace wrist {
namespace coral {
inline constexpr units::radian_t loading = 90_deg;
inline constexpr units::radian_t l1 = -56_deg;
inline constexpr units::radian_t l2 = -83_deg;
inline constexpr units::radian_t l3 = -83_deg;
inline constexpr units::radian_t l4 = -85_deg;
}  // namespace coral

inline constexpr units::radian_t algaeHold = 10_deg;
inline constexpr units::radian_t algaeProcess = -30_deg;
inline constexpr units::radian_t algaeGrab = 0_deg;
inline constexpr units::radian_t algaeNet = 80_deg;
inline constexpr units::radian_t primed = 45_deg;

inline constexpr units::radian_t outofstarting = 20_deg;
}  // namespace wrist

namespace elev {
namespace algae {
inline constexpr units::meter_t hold = 12_in;
inline constexpr units::meter_t processor = 12_in;
inline constexpr units::meter_t l2 = 20_in;
inline constexpr units::meter_t l3 = 35_in;
inline constexpr units::meter_t net = 79_in;
}  // namespace algae

namespace coral {
inline constexpr units::meter_t loading = 7_in;
inline constexpr units::meter_t l1 = 12_in;
inline constexpr units::meter_t l2 = 34_in;
inline constexpr units::meter_t l3 = 50_in;
inline constexpr units::meter_t l4 = 75_in;
}  // namespace coral

inline constexpr units::meter_t outofstarting = 12_in;
inline constexpr units::meter_t clearOfChassis = 14_in;
}  // namespace elev
}  // namespace presets
