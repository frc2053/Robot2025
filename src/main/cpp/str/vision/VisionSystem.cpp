// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/vision/VisionSystem.h"

#include <frc/geometry/Pose2d.h>

#include <vector>

using namespace str::vision;

void VisionSystem::UpdateCameraPositionVis(frc::Pose3d robotPose) {
  cameraLocations[0] = robotPose.TransformBy(consts::vision::FL_ROBOT_TO_CAM);
  cameraLocations[1] = robotPose.TransformBy(consts::vision::FR_ROBOT_TO_CAM);
  cameraLocations[2] = robotPose.TransformBy(consts::vision::BL_ROBOT_TO_CAM);
  cameraLocations[3] = robotPose.TransformBy(consts::vision::BR_ROBOT_TO_CAM);

  cameraLocationsPub.Set(cameraLocations);
}

std::vector<std::optional<Eigen::Matrix<double, 3, 1>>>
VisionSystem::GetPoseStdDevs(
    const std::vector<std::optional<photon::EstimatedRobotPose>>& poses) {
  std::vector<std::optional<Eigen::Matrix<double, 3, 1>>> allStdDevs;
  int i = 0;
  for (auto& cam : cameras) {
    if (poses[i].has_value()) {
      auto stddev =
          cam.GetEstimationStdDevs(poses[i].value().estimatedPose.ToPose2d());
      allStdDevs.push_back(stddev);
    } else {
      allStdDevs.push_back(std::nullopt);
    }
    i++;
  }
  return allStdDevs;
}

std::vector<std::optional<photon::EstimatedRobotPose>>
VisionSystem::GetCameraEstimatedPoses(frc::Pose3d robotPose) {
  std::vector<std::optional<photon::EstimatedRobotPose>> allPoses;
  for (auto& cam : cameras) {
    allPoses.push_back(cam.GetEstimatedGlobalPose(robotPose));
    allPoses.push_back(cam.LatestSingleTagPose());
  }
  return allPoses;
}

void VisionSystem::SimulationPeriodic(frc::Pose2d simRobotPose) {
  for (auto& cam : cameras) {
    cam.SimPeriodic(simRobotPose);
  }
}
