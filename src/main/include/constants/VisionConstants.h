// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/geometry/Transform3d.h>

#include <string>

namespace consts::vision {
inline const std::string FL_CAM_NAME{"str_fl_cam"};
inline const frc::Transform3d FL_ROBOT_TO_CAM{
    frc::Translation3d{0.265256_m, 0.2770_m, 0.209751_m},
    frc::Rotation3d{0_rad, -20_deg, -20_deg}};

inline const std::string FR_CAM_NAME{"str_fr_cam"};
inline const frc::Transform3d FR_ROBOT_TO_CAM{
    frc::Translation3d{0.262526_m, -0.287850_m, 0.2141_m},
    frc::Rotation3d{0_rad, -20_deg, 20_deg}};

inline const Eigen::Matrix<double, 3, 1> SINGLE_TAG_STD_DEV{.2, .2, 999999};
inline const Eigen::Matrix<double, 3, 1> MULTI_TAG_STD_DEV{0.1, 0.1, 1};
}  // namespace consts::vision
