// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/vision/Camera.h"

#include <frc/DataLogManager.h>
#include <frc/RobotBase.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "constants/Constants.h"
#include "constants/VisionConstants.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation3d.h"
#include "photon/estimation/TargetModel.h"
#include "photon/simulation/VisionTargetSim.h"
#include "units/angle.h"

using namespace str::vision;

Camera::Camera(std::string cameraName, frc::Transform3d robotToCamera,
               Eigen::Matrix<double, 3, 1> singleTagStdDev,
               Eigen::Matrix<double, 3, 1> multiTagDevs, bool simulate)
    : simulate(simulate),
      singleTagDevs(singleTagStdDev),
      multiTagDevs(multiTagDevs),
      posePub(nt->GetStructTopic<frc::Pose2d>(cameraName + "PoseEstimation")
                  .Publish()),
      stdDevXPosePub(nt->GetDoubleTopic(cameraName + "StdDevsX").Publish()),
      stdDevYPosePub(nt->GetDoubleTopic(cameraName + "StdDevsY").Publish()),
      stdDevRotPosePub(nt->GetDoubleTopic(cameraName + "StdDevsRot").Publish()),
      targetPosesPub(
          nt->GetStructArrayTopic<frc::Pose3d>(cameraName + "targetPoses")
              .Publish()),
      cornersPub(nt->GetStructArrayTopic<frc::Translation2d>(cameraName +
                                                             "targetCorners")
                     .Publish()) {
  photonEstimator = std::make_unique<photon::PhotonPoseEstimator>(
      consts::yearspecific::TAG_LAYOUT,
      photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
  camera = std::make_unique<photon::PhotonCamera>(cameraName);
  camera->SetVersionCheckEnabled(true);
  photonEstimator->SetMultiTagFallbackStrategy(
      photon::PoseStrategy::LOWEST_AMBIGUITY);

  if (simulate) {
    if (frc::RobotBase::IsSimulation()) {
      visionSim = std::make_unique<photon::VisionSystemSim>(cameraName);
      visionSim->AddAprilTags(consts::yearspecific::TAG_LAYOUT);
      cameraProps = std::make_unique<photon::SimCameraProperties>();

      if (cameraName == consts::vision::NOTE_CAM_NAME) {
        cameraProps->SetCalibration(640, 480, frc::Rotation2d{75_deg});
      } else {
        cameraProps->SetCalibration(1600, 1200, frc::Rotation2d{75_deg});
      }
      cameraProps->SetCalibError(.35, .10);
      cameraProps->SetFPS(45_Hz);
      cameraProps->SetAvgLatency(20_ms);
      cameraProps->SetLatencyStdDev(15_ms);

      cameraSim = std::make_shared<photon::PhotonCameraSim>(camera.get(),
                                                            *cameraProps.get());

      visionSim->AddCamera(cameraSim.get(), robotToCamera);
      cameraSim->EnableDrawWireframe(true);
    }
  }
}

std::optional<photon::EstimatedRobotPose> Camera::GetEstimatedGlobalPose(
    frc::Pose3d robotPose) {
  std::optional<photon::EstimatedRobotPose> visionEst;

  auto allUnread = camera->GetAllUnreadResults();

  for (const auto& result : allUnread) {
    visionEst = photonEstimator->Update(result);

    if (visionEst.has_value()) {
      posePub.Set(visionEst.value().estimatedPose.ToPose2d());
    } else {
      posePub.Set({});
    }

    const auto& targetsSpan = result.GetTargets();
    targetsCopy = std::vector<photon::PhotonTrackedTarget>(targetsSpan.begin(),
                                                           targetsSpan.end());

    std::vector<frc::Pose3d> targetPoses;
    std::vector<frc::Translation2d> cornerPxs;
    for (const auto& target : targetsCopy) {
      targetPoses.emplace_back(
          robotPose.TransformBy(photonEstimator->GetRobotToCameraTransform())
              .TransformBy(target.bestCameraToTarget));
      for (const auto& corner : target.GetDetectedCorners()) {
        cornerPxs.emplace_back(frc::Translation2d{units::meter_t{corner.x},
                                                  units::meter_t{corner.y}});
      }
    }
    targetPosesPub.Set(targetPoses);
    cornersPub.Set(cornerPxs);
  }

  return visionEst;
}

Eigen::Matrix<double, 3, 1> Camera::GetEstimationStdDevs(
    frc::Pose2d estimatedPose) {
  Eigen::Matrix<double, 3, 1> estStdDevs = singleTagDevs;
  int numTags = 0;
  units::meter_t avgDist = 0_m;
  for (const auto& tgt : targetsCopy) {
    auto tagPose =
        photonEstimator->GetFieldLayout().GetTagPose(tgt.GetFiducialId());
    if (tagPose.has_value()) {
      numTags++;
      avgDist += tagPose.value().ToPose2d().Translation().Distance(
          estimatedPose.Translation());
    }
  }
  if (numTags == 0) {
    return estStdDevs;
  }
  avgDist /= numTags;
  if (numTags > 1) {
    estStdDevs = multiTagDevs;
  }
  if (numTags == 1 && avgDist > 4_m) {
    estStdDevs =
        (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
         std::numeric_limits<double>::max(), std::numeric_limits<double>::max())
            .finished();
  } else {
    estStdDevs = estStdDevs * (1 + (avgDist.value() * avgDist.value() / 30.0));
  }

  if (estStdDevs(0) == 0 || estStdDevs(1) == 0 || estStdDevs(2) == 0) {
    frc::DataLogManager::Log("ERROR STD DEV IS ZERO!\n");
  }

  stdDevXPosePub.Set(estStdDevs(0));
  stdDevYPosePub.Set(estStdDevs(1));
  stdDevRotPosePub.Set(estStdDevs(2));

  return estStdDevs;
}

void Camera::SimPeriodic(frc::Pose2d robotSimPose) {
  if (simulate) {
    visionSim->Update(robotSimPose);
  }
}
