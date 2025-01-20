// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <units/angle.h>
#include <units/length.h>

namespace presets {
namespace wrist {
namespace coral {
inline constexpr units::radian_t l1 = 180_deg;
inline constexpr units::radian_t l2 = -150_deg;
inline constexpr units::radian_t l3 = -150_deg;
inline constexpr units::radian_t l4 = -150_deg;
}  // namespace coral

inline constexpr units::radian_t algaeProcess = -10_deg;
inline constexpr units::radian_t algaeGrab = 100_deg;
inline constexpr units::radian_t algaeNet = 100_deg;
}  // namespace wrist

namespace elev {
namespace algae {
inline constexpr units::meter_t processor = 5_in;
inline constexpr units::meter_t l2 = 12_in;
inline constexpr units::meter_t l3 = 24_in;
inline constexpr units::meter_t net = 80_in;
}  // namespace algae

namespace coral {
inline constexpr units::meter_t l1 = 12_in;
inline constexpr units::meter_t l2 = 31_in;
inline constexpr units::meter_t l3 = 46_in;
inline constexpr units::meter_t l4 = 73_in;
}  // namespace coral

}  // namespace elev
}  // namespace presets
