// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/geometry/Pose2d.h>

#include "constants/Constants.h"

namespace str::math {
static bool IsPointInsideField(const frc::Translation2d& point) {
  return point.X() >= 0_m && point.Y() >= 0_m &&
         point.X() <= consts::yearspecific::TAG_LAYOUT.GetFieldLength() &&
         point.Y() <= consts::yearspecific::TAG_LAYOUT.GetFieldWidth();
}

static bool IsRobotInsideField(units::meter_t robotTotalWidth,
                               units::meter_t robotTotalLength,
                               const frc::Pose2d& poseToCheck) {
  units::radian_t currentRotation = poseToCheck.Rotation().Radians();
  units::meter_t ox = (robotTotalWidth / 2);
  units::meter_t oy = (robotTotalLength / 2);

  frc::Translation2d flCorner{
      poseToCheck.X() + (ox * units::math::cos(currentRotation)) -
          (oy * units::math::sin(currentRotation)),
      poseToCheck.Y() + (ox * units::math::sin(currentRotation)) +
          (oy * units::math::cos(currentRotation))};

  frc::Translation2d frCorner{
      poseToCheck.X() + (ox * units::math::cos(currentRotation)) +
          (oy * units::math::sin(currentRotation)),
      poseToCheck.Y() + (ox * units::math::sin(currentRotation)) -
          (oy * units::math::cos(currentRotation))};

  frc::Translation2d blCorner{
      poseToCheck.X() - (ox * units::math::cos(currentRotation)) -
          (oy * units::math::sin(currentRotation)),
      poseToCheck.Y() - (ox * units::math::sin(currentRotation)) +
          (oy * units::math::cos(currentRotation))};

  frc::Translation2d brCorner{
      poseToCheck.X() - (ox * units::math::cos(currentRotation)) +
          (oy * units::math::sin(currentRotation)),
      poseToCheck.Y() - (ox * units::math::sin(currentRotation)) -
          (oy * units::math::cos(currentRotation))};

  return math::IsPointInsideField(flCorner) &&
         math::IsPointInsideField(frCorner) &&
         math::IsPointInsideField(blCorner) &&
         math::IsPointInsideField(brCorner);
}
}  // namespace str::math
