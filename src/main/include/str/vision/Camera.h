// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Transform3d.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
#include <photon/PhotonCamera.h>
#include <photon/estimation/VisionEstimation.h>
#include <photon/simulation/VisionSystemSim.h>
#include <photon/simulation/VisionTargetSim.h>
#include <photon/targeting/PhotonPipelineResult.h>
#include <str/vision/StrPoseEstimator.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Translation3d.h"
#include "frc/interpolation/TimeInterpolatableBuffer.h"
#include "units/angle.h"
#include "units/length.h"
#include "units/time.h"

namespace str::vision {
class Camera {
 public:
  Camera(std::string cameraName, frc::Transform3d robotToCamera,
         Eigen::Matrix<double, 3, 1> singleTagStdDev,
         Eigen::Matrix<double, 3, 1> multiTagDevs, bool simulate,
         std::function<void(const frc::Pose2d&, units::second_t,
                            const Eigen::Vector3d& stdDevs)>
             visionConsumer,
         std::function<void(const frc::Pose2d&, units::second_t,
                            const Eigen::Vector3d& stdDevs)>
             singleTagCon);
  void SimPeriodic(frc::Pose2d robotSimPose);
  void UpdatePoseEstimator(frc::Pose3d robotPose);
  std::optional<str::EstimatedRobotPose> ImuTagOnRio(
      photon::PhotonPipelineResult result);
  void AddYaw(units::radian_t yaw, units::second_t time) {
    yawBuffer.AddSample(time, yaw);
    singleTagEstimator->AddHeadingData(time, frc::Rotation2d{yaw});
    photonEstimator->AddHeadingData(time, frc::Rotation2d{yaw});
  }
  std::optional<str::EstimatedRobotPose> LatestSingleTagPose() {
    return singleTagPose;
  }

 private:
  std::array<int, 12> reefTags{6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
  Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d estimatedPose);

  bool simulate;
  std::function<void(const frc::Pose2d&, units::second_t,
                     const Eigen::Vector3d& stdDevs)>
      consumer;
  std::function<void(const frc::Pose2d&, units::second_t,
                     const Eigen::Vector3d& stdDevs)>
      singleTagConsumer;
  std::unique_ptr<str::StrPoseEstimator> photonEstimator;
  std::unique_ptr<str::StrPoseEstimator> singleTagEstimator;
  std::unique_ptr<photon::PhotonCamera> camera;
  std::unique_ptr<photon::VisionSystemSim> visionSim;
  std::unique_ptr<photon::SimCameraProperties> cameraProps;
  std::shared_ptr<photon::PhotonCameraSim> cameraSim;
  std::optional<str::EstimatedRobotPose> singleTagPose;
  frc::Transform3d robotToCam{};
  frc::TimeInterpolatableBuffer<units::radian_t> yawBuffer{1.0_s};

  photon::PhotonPipelineResult latestResult;
  std::vector<photon::PhotonTrackedTarget> targetsCopy;

  frc::Rotation3d GetCorrectedPixelRot(
      const cv::Point2d& point,
      const Eigen::Matrix<double, 3, 3>& camIntrinsics) const {
    double fx = camIntrinsics(0, 0);
    double cx = camIntrinsics(0, 2);
    double xOffset = cx - point.x;

    double fy = camIntrinsics(1, 1);
    double cy = camIntrinsics(1, 2);
    double yOffset = cy - point.y;

    frc::Rotation2d yaw{fx, xOffset};
    frc::Rotation2d pitch{fy / std::cos(std::atan(xOffset / fx)), -yOffset};
    return frc::Rotation3d{0_rad, pitch.Radians(), yaw.Radians()};
  }

  Eigen::Matrix<double, 3, 1> singleTagDevs;
  Eigen::Matrix<double, 3, 1> multiTagDevs;

  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("Vision")};
  nt::StructPublisher<frc::Pose2d> posePub;
  nt::StructPublisher<frc::Pose2d> singleTagPosePub;
  nt::DoublePublisher stdDevXPosePub;
  nt::DoublePublisher stdDevYPosePub;
  nt::DoublePublisher stdDevRotPosePub;
  nt::StructArrayPublisher<frc::Pose3d> targetPosesPub;
  nt::StructArrayPublisher<frc::Translation2d> cornersPub;
};
}  // namespace str::vision
