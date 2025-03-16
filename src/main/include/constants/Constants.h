// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>

#include "frc/Filesystem.h"
#include "frc/apriltag/AprilTag.h"
#include "frc/apriltag/AprilTagFields.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Quaternion.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Transform3d.h"
#include "frc/geometry/Translation3d.h"

namespace consts::yearspecific {
// inline const frc::AprilTagFieldLayout TAG_LAYOUT =
//     frc::AprilTagFieldLayout::LoadField(
//         frc::AprilTagField::k2025ReefscapeWelded);

inline constexpr units::inch_t CLAW_OFFSET_L = 0_in;
inline constexpr units::inch_t CLAW_OFFSET_R = 0_in;
inline constexpr frc::Transform2d CLAW_TRANS_L{0_m, CLAW_OFFSET_L,
                                               frc::Rotation2d{}};
inline constexpr frc::Transform2d CLAW_TRANS_R{0_m, CLAW_OFFSET_R,
                                               frc::Rotation2d{}};
inline const frc::AprilTagFieldLayout TAG_LAYOUT = frc::AprilTagFieldLayout(
    frc::filesystem::GetDeployDirectory() + "/reefonlytags.json");
}  // namespace consts::yearspecific
