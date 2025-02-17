// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/vision/Camera.h"

#include <frc/DataLogManager.h>
#include <frc/RobotBase.h>

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "constants/Constants.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Transform2d.h"
#include "frc/geometry/Transform3d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Translation3d.h"
#include "opencv2/core/types.hpp"
#include "photon/PhotonPoseEstimator.h"
#include "photon/targeting/PhotonTrackedTarget.h"
#include "photon/targeting/TargetCorner.h"
#include "units/angle.h"
#include "units/length.h"

using namespace str::vision;

Camera::Camera(std::string cameraName, frc::Transform3d robotToCamera,
               Eigen::Matrix<double, 3, 1> singleTagStdDev,
               Eigen::Matrix<double, 3, 1> multiTagDevs, bool simulate,
               std::function<void(const frc::Pose2d&, units::second_t,
                                  const Eigen::Vector3d& stdDevs)>
                   visionConsumer,
               std::function<void(const frc::Pose2d&, units::second_t,
                                  const Eigen::Vector3d& stdDevs)>
                   singleTagCon)
    : simulate(simulate),
      consumer(visionConsumer),
      singleTagConsumer(singleTagCon),
      robotToCam(robotToCamera),
      singleTagDevs(singleTagStdDev),
      multiTagDevs(multiTagDevs),
      posePub(nt->GetStructTopic<frc::Pose2d>(cameraName + "PoseEstimation")
                  .Publish()),
      singleTagPosePub(
          nt->GetStructTopic<frc::Pose2d>(cameraName + "SingleTagPose")
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
  camera->SetVersionCheckEnabled(false);
  photonEstimator->SetMultiTagFallbackStrategy(
      photon::PoseStrategy::LOWEST_AMBIGUITY);

  if (simulate) {
    if (frc::RobotBase::IsSimulation()) {
      visionSim = std::make_unique<photon::VisionSystemSim>(cameraName);
      visionSim->AddAprilTags(consts::yearspecific::TAG_LAYOUT);
      cameraProps = std::make_unique<photon::SimCameraProperties>();

      cameraProps->SetCalibration(1280, 960, frc::Rotation2d{90_deg});

      cameraProps->SetCalibError(.7, .10);
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

void Camera::UpdatePoseEstimator(frc::Pose3d robotPose) {
  std::optional<photon::EstimatedRobotPose> visionEst;

  auto allUnread = camera->GetAllUnreadResults();

  for (const auto& result : allUnread) {
    visionEst = photonEstimator->Update(result);
    singleTagPose = ImuTagOnRio(result);

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

    // TRICKING PHOTON STRATS
    // WE ONLY WANT SINGLE TAG RESULTS TO BE ADDED IF THEY ARE OUR TRIG
    if (result.targets.size() == 1) {
      if (singleTagPose->strategy ==
          photon::PoseStrategy::CLOSEST_TO_CAMERA_HEIGHT) {
        if (singleTagPose.has_value()) {
          singleTagPosePub.Set(singleTagPose->estimatedPose.ToPose2d());
          singleTagConsumer(
              singleTagPose->estimatedPose.ToPose2d(), singleTagPose->timestamp,
              GetEstimationStdDevs(singleTagPose->estimatedPose.ToPose2d()));
        }
      }
    } else {
      if (visionEst.has_value()) {
        consumer(visionEst->estimatedPose.ToPose2d(), visionEst->timestamp,
                 GetEstimationStdDevs(visionEst->estimatedPose.ToPose2d()));
      }
    }
  }
}

std::optional<photon::EstimatedRobotPose> Camera::ImuTagOnRio(
    photon::PhotonPipelineResult result) {
  if (result.HasTargets() && result.GetTargets().size() == 1 &&
      camera->GetCameraMatrix().has_value()) {
    photon::PhotonTrackedTarget target = result.GetBestTarget();
    int tagId = target.GetFiducialId();
    std::optional<frc::Pose3d> tagPose =
        consts::yearspecific::TAG_LAYOUT.GetTagPose(tagId);
    if (!tagPose.has_value()) {
      return {};
    }

    std::vector<photon::TargetCorner> targetCorners =
        target.GetDetectedCorners();
    double sumX = 0.0;
    double sumY = 0.0;
    for (const auto& t : targetCorners) {
      sumX = sumX + t.x;
      sumY = sumY + t.y;
    }

    cv::Point2d tagCenter{sumX / 4.0, sumY / 4.0};
    frc::Rotation3d tagAngle =
        GetCorrectedPixelRot(tagCenter, camera->GetCameraMatrix().value());

    frc::Transform3d best = target.GetBestCameraToTarget();

    units::meter_t distance = best.Translation().Norm();

    units::meter_t distance2d =
        distance * units::math::cos(-robotToCam.Rotation().Y() - tagAngle.Y());

    std::optional<units::radian_t> headingState =
        yawBuffer.Sample(result.GetTimestamp());
    if (!headingState.has_value()) {
      return {};
    }

    frc::Rotation2d robotHeading{headingState.value()};

    frc::Rotation2d camToTagRotation =
        (robotHeading + robotToCam.Rotation().ToRotation2d()) +
        frc::Rotation2d{tagAngle.Z()};

    frc::Pose2d fieldToCameraPose =
        frc::Pose2d{tagPose->ToPose2d().Translation(),
                    camToTagRotation +
                        frc::Rotation2d{units::radian_t{std::numbers::pi}}}
            .TransformBy(frc::Transform2d{distance2d, 0_m, frc::Rotation2d{}});

    frc::Translation2d fieldToCameraTranslation =
        fieldToCameraPose.Translation();

    frc::Pose2d robotPose =
        frc::Pose2d{fieldToCameraTranslation,
                    robotHeading + robotToCam.Rotation().ToRotation2d()}
            .TransformBy(frc::Transform2d{
                frc::Pose2d{robotToCam.X(), robotToCam.Y(),
                            robotToCam.Rotation().ToRotation2d()},
                frc::Pose2d{}});

    robotPose = frc::Pose2d{robotPose.Translation(), robotHeading};

    return photon::EstimatedRobotPose{
        frc::Pose3d{robotPose}, result.GetTimestamp(), result.GetTargets(),
        photon::PoseStrategy::CLOSEST_TO_CAMERA_HEIGHT};
  } else {
    return {};
  }
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
