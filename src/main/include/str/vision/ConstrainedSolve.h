// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <vector>

#include "frc/apriltag/AprilTagFieldLayout.h"
#include "photon/constrained_solvepnp/wrap/casadi_wrapper.h"
#include "photon/estimation/OpenCVHelp.h"
#include "photon/estimation/TargetModel.h"
#include "photon/targeting/PhotonTrackedTarget.h"
#include "photon/targeting/PnpResult.h"

static std::optional<photon::PnpResult> EstimateRobotPoseConstrainedSolvePNP(
    const Eigen::Matrix<double, 3, 3>& cameraMatrix,
    const Eigen::Matrix<double, 8, 1>& distCoeffs,
    const std::vector<photon::PhotonTrackedTarget>& visTags,
    const frc::Transform3d& robot2Camera, const frc::Pose3d& robotPoseSeed,
    const frc::AprilTagFieldLayout& layout, const photon::TargetModel& tagModel,
    bool headingFree, frc::Rotation2d gyroTheta, double gyroErrorScaleFac) {
  if (visTags.size() == 0) {
    return photon::PnpResult();
  }

  std::vector<photon::TargetCorner> corners{};
  std::vector<frc::AprilTag> knownTags{};

  for (const auto& tgt : visTags) {
    int id = tgt.GetFiducialId();
    auto maybePose = layout.GetTagPose(id);
    if (maybePose) {
      knownTags.emplace_back(frc::AprilTag{id, maybePose.value()});
      auto currentCorners = tgt.GetDetectedCorners();
      corners.insert(corners.end(), currentCorners.begin(),
                     currentCorners.end());
    }
  }
  if (knownTags.size() == 0 || corners.size() == 0 || corners.size() % 4 != 0) {
    return photon::PnpResult{};
  }

  std::vector<cv::Point2f> points =
      photon::OpenCVHelp::CornersToPoints(corners);

  cv::Mat cameraMat(cameraMatrix.rows(), cameraMatrix.cols(), CV_64F);
  cv::eigen2cv(cameraMatrix, cameraMat);
  cv::Mat distCoeffsMat(distCoeffs.rows(), distCoeffs.cols(), CV_64F);
  cv::eigen2cv(distCoeffs, distCoeffsMat);

  cv::undistortImagePoints(points, points, cameraMat, distCoeffsMat);

  Eigen::Matrix4d robotToCameraBase{
      (Eigen::Matrix4d() << 0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1)
          .finished()};
  Eigen::Matrix<constrained_solvepnp::casadi_real, 4, 4, Eigen::ColMajor>
      robotToCamera{robot2Camera.ToMatrix() * robotToCameraBase};

  Eigen::Matrix<constrained_solvepnp::casadi_real, 2, Eigen::Dynamic,
                Eigen::ColMajor>
      pointObservations{};
  pointObservations.resize(2, points.size());
  for (size_t i = 0; i < points.size(); i++) {
    pointObservations(0, i) = points[i].x;
    pointObservations(1, i) = points[i].y;
  }

  std::vector<frc::Translation3d> objectTrls{};
  for (const auto& tag : knownTags) {
    auto verts = tagModel.GetFieldVertices(tag.pose);
    objectTrls.insert(objectTrls.end(), verts.begin(), verts.end());
  }

  Eigen::Matrix<constrained_solvepnp::casadi_real, 4, Eigen::Dynamic,
                Eigen::ColMajor>
      field2points{};
  field2points.resize(4, objectTrls.size());
  for (size_t i = 0; i < objectTrls.size(); i++) {
    field2points(0, i) = objectTrls[i].X().value();
    field2points(1, i) = objectTrls[i].Y().value();
    field2points(2, i) = objectTrls[i].Z().value();
    field2points(3, i) = 1;
  }

  frc::Pose2d guess2 = robotPoseSeed.ToPose2d();

  constrained_solvepnp::CameraCalibration cameraCal{
      cameraMatrix(0, 0),
      cameraMatrix(1, 1),
      cameraMatrix(0, 2),
      cameraMatrix(1, 2),
  };

  Eigen::Matrix<constrained_solvepnp::casadi_real, 3, 1> guessMat{
      guess2.X().value(), guess2.Y().value(),
      guess2.Rotation().Radians().value()};

  wpi::expected<constrained_solvepnp::RobotStateMat,
                sleipnir::SolverExitCondition>
      result = constrained_solvepnp::do_optimization(
          headingFree, knownTags.size(), cameraCal, robotToCamera, guessMat,
          field2points, pointObservations, gyroTheta.Radians().value(),
          gyroErrorScaleFac);

  if (!result.has_value()) {
    return {};
  } else {
    photon::PnpResult res{};

    res.best = frc::Transform3d{frc::Transform2d{
        units::meter_t{result.value()[0]}, units::meter_t{result.value()[1]},
        frc::Rotation2d{units::radian_t{result.value()[2]}}}};

    return res;
  }
}
