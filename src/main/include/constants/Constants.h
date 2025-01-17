// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>

#include "frc/apriltag/AprilTagFields.h"

namespace consts::yearspecific {
inline const frc::AprilTagFieldLayout TAG_LAYOUT =
    frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025Reefscape);
}  // namespace consts::yearspecific
