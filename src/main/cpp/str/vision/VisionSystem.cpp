// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/vision/VisionSystem.h"

#include <frc/geometry/Pose2d.h>

#include <vector>
#include "constants/VisionConstants.h"

using namespace str::vision;

VisionSystem::VisionSystem(
    std::function<void(const frc::Pose2d&, units::second_t,
                       const Eigen::Vector3d& stdDevs)>
        visionConsumer,
    std::function<void(const frc::Pose2d&, units::second_t,
                       const Eigen::Vector3d& stdDevs)>
        singleTagConsumer)
    : cameras{
          Camera{consts::vision::FL_CAM_NAME, consts::vision::FL_ROBOT_TO_CAM,
                 consts::vision::SINGLE_TAG_STD_DEV,
                 consts::vision::MULTI_TAG_STD_DEV, true, visionConsumer,
                 singleTagConsumer},
          Camera{consts::vision::FR_CAM_NAME, consts::vision::FR_ROBOT_TO_CAM,
                 consts::vision::SINGLE_TAG_STD_DEV,
                 consts::vision::MULTI_TAG_STD_DEV, false, visionConsumer,
                 singleTagConsumer},
          // Dont let back cameras contribute to singleTag
          Camera{consts::vision::BL_CAM_NAME, consts::vision::BL_ROBOT_TO_CAM,
                 consts::vision::SINGLE_TAG_STD_DEV,
                 consts::vision::MULTI_TAG_STD_DEV, false, visionConsumer,
                 [](const frc::Pose2d&, units::second_t,
                    const Eigen::Vector3d& stdDevs) {}},
          Camera{consts::vision::BR_CAM_NAME, consts::vision::BR_ROBOT_TO_CAM,
                 consts::vision::SINGLE_TAG_STD_DEV,
                 consts::vision::MULTI_TAG_STD_DEV, false, visionConsumer,
                 [](const frc::Pose2d&, units::second_t,
                    const Eigen::Vector3d& stdDevs) {}}} {}

void VisionSystem::UpdateCameraPositionVis(frc::Pose3d robotPose) {
  cameraLocations[0] = robotPose.TransformBy(consts::vision::FL_ROBOT_TO_CAM);
  cameraLocations[1] = robotPose.TransformBy(consts::vision::FR_ROBOT_TO_CAM);
  cameraLocations[2] = robotPose.TransformBy(consts::vision::BL_ROBOT_TO_CAM);
  cameraLocations[3] = robotPose.TransformBy(consts::vision::BR_ROBOT_TO_CAM);

  cameraLocationsPub.Set(cameraLocations);
}

void VisionSystem::UpdatePoseEstimators(frc::Pose3d robotPose) {
  for (auto& cam : cameras) {
    cam.UpdatePoseEstimator(robotPose);
  }
}

void VisionSystem::SimulationPeriodic(frc::Pose2d simRobotPose) {
  for (auto& cam : cameras) {
    cam.SimPeriodic(simRobotPose);
  }
}
