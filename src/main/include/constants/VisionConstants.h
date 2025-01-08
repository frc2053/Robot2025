// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/geometry/Transform3d.h>

#include <string>

namespace consts::vision {
inline const std::string FL_CAM_NAME{"str_fl_cam"};
inline const frc::Transform3d FL_ROBOT_TO_CAM{
    frc::Translation3d{6.773802_in, 11.2301_in, 8.495934_in},
    frc::Rotation3d{0_rad, -28.125_deg, 30_deg}};

inline const std::string FR_CAM_NAME{"str_fr_cam"};
inline const frc::Transform3d FR_ROBOT_TO_CAM{
    frc::Translation3d{6.773802_in, -11.2301_in, 8.495934_in},
    frc::Rotation3d{0_rad, -28.125_deg, -30_deg}};

inline const std::string BL_CAM_NAME{"str_bl_cam"};
inline const frc::Transform3d BL_ROBOT_TO_CAM{
    frc::Translation3d{-6.773802_in, 11.2301_in, 8.495934_in},
    frc::Rotation3d{0_rad, -28.125_deg, 150_deg}};

inline const std::string BR_CAM_NAME{"str_br_cam"};
inline const frc::Transform3d BR_ROBOT_TO_CAM{
    frc::Translation3d{-6.773802_in, -11.2301_in, 8.495934_in},
    frc::Rotation3d{0_rad, -28.125_deg, -150_deg}};

inline const Eigen::Matrix<double, 3, 1> SINGLE_TAG_STD_DEV{4, 4, 8};
inline const Eigen::Matrix<double, 3, 1> MULTI_TAG_STD_DEV{0.5, 0.5, 1};

inline const std::string NOTE_CAM_NAME{"note_cam"};
inline const frc::Transform3d ROBOT_TO_NOTE_CAM{
    frc::Translation3d{-15.664852_in, 0_in, 9.437251_in},
    frc::Rotation3d{0_rad, 20_deg, 180_deg}};
}  // namespace consts::vision
