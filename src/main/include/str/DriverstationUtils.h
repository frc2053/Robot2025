// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/DriverStation.h>

#include <functional>

namespace str {

static bool IsOnRed() {
  auto ally = frc::DriverStation::GetAlliance();
  if (ally) {
    return ally.value() == frc::DriverStation::Alliance::kRed;
  }
  return false;
}

template <typename T>
static T NegateIfRed(T input) {
  return IsOnRed() ? input * -1 : input;
}
}  // namespace str
