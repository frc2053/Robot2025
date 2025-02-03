// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <units/angle.h>
#include <units/length.h>

namespace presets {
namespace wrist {
namespace coral {
inline constexpr units::radian_t loading = 35_deg;
inline constexpr units::radian_t l1 = 180_deg;
inline constexpr units::radian_t l2 = 200_deg;
inline constexpr units::radian_t l3 = 200_deg;
inline constexpr units::radian_t l4 = 205_deg;
}  // namespace coral

inline constexpr units::radian_t algaeHold = 150_deg;
inline constexpr units::radian_t algaeProcess = 150_deg;
inline constexpr units::radian_t algaeGrab = 100_deg;
inline constexpr units::radian_t algaeNet = 100_deg;
inline constexpr units::radian_t outofstarting = 90_deg;
}  // namespace wrist

namespace elev {
namespace algae {
inline constexpr units::meter_t hold = 12_in;
inline constexpr units::meter_t processor = 12_in;
inline constexpr units::meter_t l2 = 12_in;
inline constexpr units::meter_t l3 = 24_in;
inline constexpr units::meter_t net = 80_in;
}  // namespace algae

namespace coral {
inline constexpr units::meter_t loading = 7_in;
inline constexpr units::meter_t l1 = 12_in;
inline constexpr units::meter_t l2 = 29_in;
inline constexpr units::meter_t l3 = 44_in;
inline constexpr units::meter_t l4 = 75_in;
}  // namespace coral

inline constexpr units::meter_t outofstarting = 12_in;
}  // namespace elev
}  // namespace presets
