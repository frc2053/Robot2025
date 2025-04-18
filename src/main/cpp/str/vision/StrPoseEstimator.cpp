// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/vision/StrPoseEstimator.h"

#include <frc/Errors.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <hal/FRCUsageReporting.h>
#include <units/math.h>
#include <units/time.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include "frc/apriltag/AprilTagFieldLayout.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Translation3d.h"
#include "photon/PhotonCamera.h"
#include "photon/estimation/TargetModel.h"
#include "photon/targeting/PhotonPipelineResult.h"
#include "photon/targeting/PhotonTrackedTarget.h"
#include "photon/targeting/PnpResult.h"
#include "str/vision/ConstrainedSolve.h"
#include "units/angle.h"

#define OPENCV_DISABLE_EIGEN_TENSOR_SUPPORT
#include <opencv2/core/eigen.hpp>

namespace str {

namespace detail {
cv::Point3d ToPoint3d(const frc::Translation3d& translation);
std::optional<std::array<cv::Point3d, 4>> CalcTagCorners(
    int tagID, const frc::AprilTagFieldLayout& aprilTags);
frc::Pose3d ToPose3d(const cv::Mat& tvec, const cv::Mat& rvec);
cv::Point3d TagCornerToObjectPoint(units::meter_t cornerX,
                                   units::meter_t cornerY, frc::Pose3d tagPose);
}  // namespace detail

StrPoseEstimator::StrPoseEstimator(frc::AprilTagFieldLayout tags,
                                   PoseStrategy strat,
                                   frc::Transform3d robotToCamera)
    : aprilTags(tags),
      strategy(strat),
      m_robotToCamera(robotToCamera),
      lastPose(frc::Pose3d()),
      referencePose(frc::Pose3d()),
      poseCacheTimestamp(-1_s) {
  HAL_Report(HALUsageReporting::kResourceType_PhotonPoseEstimator,
             InstanceCount);
  InstanceCount++;
}

void StrPoseEstimator::SetMultiTagFallbackStrategy(PoseStrategy strategy) {
  if (strategy == MULTI_TAG_PNP_ON_COPROCESSOR ||
      strategy == MULTI_TAG_PNP_ON_RIO) {
    FRC_ReportError(
        frc::warn::Warning,
        "Fallback cannot be set to MULTI_TAG_PNP! Setting to lowest ambiguity",
        "");
    strategy = LOWEST_AMBIGUITY;
  }
  if (this->multiTagFallbackStrategy != strategy) {
    InvalidatePoseCache();
  }
  multiTagFallbackStrategy = strategy;
}

std::optional<EstimatedRobotPose> StrPoseEstimator::Update(
    const photon::PhotonPipelineResult& result,
    std::optional<photon::PhotonCamera::CameraMatrix> cameraMatrixData,
    std::optional<photon::PhotonCamera::DistortionMatrix> cameraDistCoeffs) {
  // Time in the past -- give up, since the following if expects times > 0
  if (result.GetTimestamp() < 0_s) {
    FRC_ReportError(frc::warn::Warning,
                    "Result timestamp was reported in the past!");
    return std::nullopt;
  }

  // If the pose cache timestamp was set, and the result is from the same
  // timestamp, return an empty result
  if (poseCacheTimestamp > 0_s &&
      units::math::abs(poseCacheTimestamp - result.GetTimestamp()) < 0.001_ms) {
    return std::nullopt;
  }

  // Remember the timestamp of the current result used
  poseCacheTimestamp = result.GetTimestamp();

  // If no targets seen, trivial case -- return empty result
  if (!result.HasTargets()) {
    return std::nullopt;
  }

  return Update(result, cameraMatrixData, cameraDistCoeffs, std::nullopt,
                this->strategy);
}

std::optional<EstimatedRobotPose> StrPoseEstimator::Update(
    const photon::PhotonPipelineResult& result,
    std::optional<photon::PhotonCamera::CameraMatrix> cameraMatrixData,
    std::optional<photon::PhotonCamera::DistortionMatrix> cameraDistCoeffs,
    std::optional<ConstrainedSolvepnpParams> constrainedPnpParams) {
  // Time in the past -- give up, since the following if expects times > 0
  if (result.GetTimestamp() < 0_s) {
    FRC_ReportError(frc::warn::Warning,
                    "Result timestamp was reported in the past!");
    return std::nullopt;
  }

  // If the pose cache timestamp was set, and the result is from the same
  // timestamp, return an empty result
  if (poseCacheTimestamp > 0_s &&
      units::math::abs(poseCacheTimestamp - result.GetTimestamp()) < 0.001_ms) {
    return std::nullopt;
  }

  // Remember the timestamp of the current result used
  poseCacheTimestamp = result.GetTimestamp();

  // If no targets seen, trivial case -- return empty result
  if (!result.HasTargets()) {
    return std::nullopt;
  }

  return Update(result, cameraMatrixData, cameraDistCoeffs,
                constrainedPnpParams, this->strategy);
}

std::optional<EstimatedRobotPose> StrPoseEstimator::Update(
    const photon::PhotonPipelineResult& result,
    std::optional<photon::PhotonCamera::CameraMatrix> cameraMatrixData,
    std::optional<photon::PhotonCamera::DistortionMatrix> cameraDistCoeffs,
    std::optional<ConstrainedSolvepnpParams> constrainedPnpParams,
    PoseStrategy strategy) {
  std::optional<EstimatedRobotPose> ret = std::nullopt;

  switch (strategy) {
    case LOWEST_AMBIGUITY:
      ret = LowestAmbiguityStrategy(result);
      break;
    case CLOSEST_TO_CAMERA_HEIGHT:
      ret = ClosestToCameraHeightStrategy(result);
      break;
    case CLOSEST_TO_REFERENCE_POSE:
      ret = ClosestToReferencePoseStrategy(result);
      break;
    case CLOSEST_TO_LAST_POSE:
      SetReferencePose(lastPose);
      ret = ClosestToReferencePoseStrategy(result);
      break;
    case AVERAGE_BEST_TARGETS:
      ret = AverageBestTargetsStrategy(result);
      break;
    case MULTI_TAG_PNP_ON_COPROCESSOR:
      ret = MultiTagOnCoprocStrategy(result);
      break;
    case MULTI_TAG_PNP_ON_RIO:
      if (cameraMatrixData && cameraDistCoeffs) {
        ret = MultiTagOnRioStrategy(result, cameraMatrixData, cameraDistCoeffs);
      } else {
        FRC_ReportError(frc::warn::Warning,
                        "No camera calibration provided to multi-tag-on-rio!",
                        "");
      }
      break;
    case CONSTRAINED_SOLVEPNP:
      ret = ConstrainedPnpStrategy(result, cameraMatrixData, cameraDistCoeffs,
                                   constrainedPnpParams);
      break;
    case PNP_DISTANCE_TRIG_SOLVE:
      ret = PnpDistanceTrigSolveStrategy(result);
      break;
    default:
      FRC_ReportError(frc::warn::Warning, "Invalid Pose Strategy selected!",
                      "");
      ret = std::nullopt;
  }

  if (ret) {
    lastPose = ret->estimatedPose;
  }
  return ret;
}

std::optional<EstimatedRobotPose> StrPoseEstimator::LowestAmbiguityStrategy(
    photon::PhotonPipelineResult result) {
  double lowestAmbiguityScore = std::numeric_limits<double>::infinity();
  auto targets = result.GetTargets();
  auto foundIt = targets.end();
  for (auto it = targets.begin(); it != targets.end(); ++it) {
    if (it->GetPoseAmbiguity() < lowestAmbiguityScore) {
      foundIt = it;
      lowestAmbiguityScore = it->GetPoseAmbiguity();
    }
  }

  if (foundIt == targets.end()) {
    return std::nullopt;
  }

  auto& bestTarget = *foundIt;

  std::optional<frc::Pose3d> fiducialPose =
      aprilTags.GetTagPose(bestTarget.GetFiducialId());
  if (!fiducialPose) {
    FRC_ReportError(frc::warn::Warning,
                    "Tried to get pose of unknown April Tag: {}",
                    bestTarget.GetFiducialId());
    return std::nullopt;
  }

  return EstimatedRobotPose{
      fiducialPose->TransformBy(bestTarget.GetBestCameraToTarget().Inverse())
          .TransformBy(m_robotToCamera.Inverse()),
      result.GetTimestamp(), result.GetTargets(), LOWEST_AMBIGUITY};
}

std::optional<EstimatedRobotPose>
StrPoseEstimator::ClosestToCameraHeightStrategy(
    photon::PhotonPipelineResult result) {
  units::meter_t smallestHeightDifference =
      units::meter_t(std::numeric_limits<double>::infinity());

  std::optional<EstimatedRobotPose> pose = std::nullopt;

  for (auto& target : result.GetTargets()) {
    std::optional<frc::Pose3d> fiducialPose =
        aprilTags.GetTagPose(target.GetFiducialId());
    if (!fiducialPose) {
      FRC_ReportError(frc::warn::Warning,
                      "Tried to get pose of unknown April Tag: {}",
                      target.GetFiducialId());
      continue;
    }
    frc::Pose3d const targetPose = *fiducialPose;

    units::meter_t const alternativeDifference = units::math::abs(
        m_robotToCamera.Z() -
        targetPose.TransformBy(target.GetAlternateCameraToTarget().Inverse())
            .Z());

    units::meter_t const bestDifference = units::math::abs(
        m_robotToCamera.Z() -
        targetPose.TransformBy(target.GetBestCameraToTarget().Inverse()).Z());

    if (alternativeDifference < smallestHeightDifference) {
      smallestHeightDifference = alternativeDifference;
      pose = EstimatedRobotPose{
          targetPose.TransformBy(target.GetAlternateCameraToTarget().Inverse())
              .TransformBy(m_robotToCamera.Inverse()),
          result.GetTimestamp(), result.GetTargets(), CLOSEST_TO_CAMERA_HEIGHT};
    }
    if (bestDifference < smallestHeightDifference) {
      smallestHeightDifference = bestDifference;
      pose = EstimatedRobotPose{
          targetPose.TransformBy(target.GetBestCameraToTarget().Inverse())
              .TransformBy(m_robotToCamera.Inverse()),
          result.GetTimestamp(), result.GetTargets(), CLOSEST_TO_CAMERA_HEIGHT};
    }
  }

  return pose;
}

std::optional<EstimatedRobotPose>
StrPoseEstimator::ClosestToReferencePoseStrategy(
    photon::PhotonPipelineResult result) {
  units::meter_t smallestDifference =
      units::meter_t(std::numeric_limits<double>::infinity());
  units::second_t stateTimestamp = units::second_t(0);
  frc::Pose3d pose = lastPose;

  auto targets = result.GetTargets();
  for (auto& target : targets) {
    std::optional<frc::Pose3d> fiducialPose =
        aprilTags.GetTagPose(target.GetFiducialId());
    if (!fiducialPose) {
      FRC_ReportError(frc::warn::Warning,
                      "Tried to get pose of unknown April Tag: {}",
                      target.GetFiducialId());
      continue;
    }
    frc::Pose3d targetPose = fiducialPose.value();

    const auto altPose =
        targetPose.TransformBy(target.GetAlternateCameraToTarget().Inverse())
            .TransformBy(m_robotToCamera.Inverse());
    const auto bestPose =
        targetPose.TransformBy(target.GetBestCameraToTarget().Inverse())
            .TransformBy(m_robotToCamera.Inverse());

    units::meter_t const alternativeDifference = units::math::abs(
        referencePose.Translation().Distance(altPose.Translation()));
    units::meter_t const bestDifference = units::math::abs(
        referencePose.Translation().Distance(bestPose.Translation()));
    if (alternativeDifference < smallestDifference) {
      smallestDifference = alternativeDifference;
      pose = altPose;
      stateTimestamp = result.GetTimestamp();
    }

    if (bestDifference < smallestDifference) {
      smallestDifference = bestDifference;
      pose = bestPose;
      stateTimestamp = result.GetTimestamp();
    }
  }

  return EstimatedRobotPose{pose, stateTimestamp, result.GetTargets(),
                            CLOSEST_TO_REFERENCE_POSE};
}

std::optional<std::array<cv::Point3d, 4>> detail::CalcTagCorners(
    int tagID, const frc::AprilTagFieldLayout& aprilTags) {
  if (auto tagPose = aprilTags.GetTagPose(tagID); tagPose.has_value()) {
    return std::array{TagCornerToObjectPoint(-3_in, -3_in, *tagPose),
                      TagCornerToObjectPoint(+3_in, -3_in, *tagPose),
                      TagCornerToObjectPoint(+3_in, +3_in, *tagPose),
                      TagCornerToObjectPoint(-3_in, +3_in, *tagPose)};
  } else {
    return std::nullopt;
  }
}

cv::Point3d detail::ToPoint3d(const frc::Translation3d& translation) {
  return cv::Point3d(-translation.Y().value(), -translation.Z().value(),
                     +translation.X().value());
}

cv::Point3d detail::TagCornerToObjectPoint(units::meter_t cornerX,
                                           units::meter_t cornerY,
                                           frc::Pose3d tagPose) {
  frc::Translation3d cornerTrans =
      tagPose.Translation() +
      frc::Translation3d(0.0_m, cornerX, cornerY).RotateBy(tagPose.Rotation());
  return ToPoint3d(cornerTrans);
}

frc::Pose3d detail::ToPose3d(const cv::Mat& tvec, const cv::Mat& rvec) {
  using namespace frc;
  using namespace units;

  cv::Mat R;
  cv::Rodrigues(rvec, R);  // R is 3x3

  R = R.t();                  // rotation of inverse
  cv::Mat tvecI = -R * tvec;  // translation of inverse

  Eigen::Matrix<double, 3, 1> tv;
  tv[0] = +tvecI.at<double>(2, 0);
  tv[1] = -tvecI.at<double>(0, 0);
  tv[2] = -tvecI.at<double>(1, 0);
  Eigen::Matrix<double, 3, 1> rv;
  rv[0] = +rvec.at<double>(2, 0);
  rv[1] = -rvec.at<double>(0, 0);
  rv[2] = +rvec.at<double>(1, 0);

  return Pose3d(Translation3d(meter_t{tv[0]}, meter_t{tv[1]}, meter_t{tv[2]}),
                Rotation3d(rv));
}

std::optional<EstimatedRobotPose> StrPoseEstimator::MultiTagOnCoprocStrategy(
    photon::PhotonPipelineResult result) {
  if (!result.MultiTagResult()) {
    return Update(result, this->multiTagFallbackStrategy);
  }

  const auto field2camera = result.MultiTagResult()->estimatedPose.best;

  const auto fieldToRobot =
      frc::Pose3d() + field2camera + m_robotToCamera.Inverse();
  return str::EstimatedRobotPose(fieldToRobot, result.GetTimestamp(),
                                 result.GetTargets(),
                                 MULTI_TAG_PNP_ON_COPROCESSOR);
}

std::optional<EstimatedRobotPose> StrPoseEstimator::MultiTagOnRioStrategy(
    photon::PhotonPipelineResult result,
    std::optional<photon::PhotonCamera::CameraMatrix> camMat,
    std::optional<photon::PhotonCamera::DistortionMatrix> distCoeffs) {
  using namespace frc;

  if (!camMat || !distCoeffs) {
    FRC_ReportError(frc::warn::Warning,
                    "No camera calibration data provided to "
                    "StrPoseEstimator::MultiTagOnRioStrategy!",
                    "");
    return Update(result, this->multiTagFallbackStrategy);
  }

  // Need at least 2 targets
  if (!result.HasTargets() || result.GetTargets().size() < 2) {
    return Update(result, this->multiTagFallbackStrategy);
  }

  auto const targets = result.GetTargets();

  // List of corners mapped from 3d space (meters) to the 2d camera screen
  // (pixels).
  std::vector<cv::Point3f> objectPoints;
  std::vector<cv::Point2f> imagePoints;

  // Add all target corners to main list of corners
  for (auto target : targets) {
    int id = target.GetFiducialId();
    if (auto const tagCorners = detail::CalcTagCorners(id, aprilTags);
        tagCorners.has_value()) {
      auto const targetCorners = target.GetDetectedCorners();
      for (size_t cornerIdx = 0; cornerIdx < 4; ++cornerIdx) {
        imagePoints.emplace_back(targetCorners[cornerIdx].x,
                                 targetCorners[cornerIdx].y);
        objectPoints.emplace_back((*tagCorners)[cornerIdx]);
      }
    }
  }

  // We should only do multi-tag if at least 2 tags (* 4 corners/tag)
  if (imagePoints.size() < 8) {
    return Update(result, this->multiTagFallbackStrategy);
  }

  // Output mats for results
  cv::Mat const rvec(3, 1, cv::DataType<double>::type);
  cv::Mat const tvec(3, 1, cv::DataType<double>::type);

  {
    cv::Mat cameraMatCV(camMat->rows(), camMat->cols(), CV_64F);
    cv::eigen2cv(*camMat, cameraMatCV);
    cv::Mat distCoeffsMatCV(distCoeffs->rows(), distCoeffs->cols(), CV_64F);
    cv::eigen2cv(*distCoeffs, distCoeffsMatCV);

    cv::solvePnP(objectPoints, imagePoints, cameraMatCV, distCoeffsMatCV, rvec,
                 tvec, false, cv::SOLVEPNP_SQPNP);
  }

  const Pose3d pose = detail::ToPose3d(tvec, rvec);

  return str::EstimatedRobotPose(pose.TransformBy(m_robotToCamera.Inverse()),
                                 result.GetTimestamp(), result.GetTargets(),
                                 MULTI_TAG_PNP_ON_RIO);
}

std::optional<EstimatedRobotPose> StrPoseEstimator::AverageBestTargetsStrategy(
    photon::PhotonPipelineResult result) {
  std::vector<std::pair<frc::Pose3d, std::pair<double, units::second_t>>>
      tempPoses;
  double totalAmbiguity = 0;

  auto targets = result.GetTargets();
  for (auto& target : targets) {
    std::optional<frc::Pose3d> fiducialPose =
        aprilTags.GetTagPose(target.GetFiducialId());
    if (!fiducialPose) {
      FRC_ReportError(frc::warn::Warning,
                      "Tried to get pose of unknown April Tag: {}",
                      target.GetFiducialId());
      continue;
    }

    frc::Pose3d targetPose = fiducialPose.value();
    // Ambiguity = 0, use that pose
    if (target.GetPoseAmbiguity() == 0) {
      return EstimatedRobotPose{
          targetPose.TransformBy(target.GetBestCameraToTarget().Inverse())
              .TransformBy(m_robotToCamera.Inverse()),
          result.GetTimestamp(), result.GetTargets(), AVERAGE_BEST_TARGETS};
    }
    totalAmbiguity += 1. / target.GetPoseAmbiguity();

    tempPoses.push_back(std::make_pair(
        targetPose.TransformBy(target.GetBestCameraToTarget().Inverse()),
        std::make_pair(target.GetPoseAmbiguity(), result.GetTimestamp())));
  }

  frc::Translation3d transform = frc::Translation3d();
  frc::Rotation3d rotation = frc::Rotation3d();

  for (std::pair<frc::Pose3d, std::pair<double, units::second_t>>& pair :
       tempPoses) {
    double const weight = (1. / pair.second.first) / totalAmbiguity;
    transform = transform + pair.first.Translation() * weight;
    rotation = rotation + pair.first.Rotation() * weight;
  }

  return EstimatedRobotPose{frc::Pose3d(transform, rotation),
                            result.GetTimestamp(), result.GetTargets(),
                            AVERAGE_BEST_TARGETS};
}

void StrPoseEstimator::AddHeadingData(units::second_t timestamp,
                                      const frc::Rotation2d& heading) {
  headingBuffer.AddSample(timestamp, heading.Radians());
}

std::optional<EstimatedRobotPose>
StrPoseEstimator::PnpDistanceTrigSolveStrategy(
    photon::PhotonPipelineResult result) {
  photon::PhotonTrackedTarget bestTarget = result.GetBestTarget();

  if (bestTarget.fiducialId == -1) {
    return {};
  }

  if (!headingBuffer.Sample(result.GetTimestamp()).has_value()) {
    return {};
  }

  frc::Rotation2d headingSample{
      headingBuffer.Sample(result.GetTimestamp()).value()};

  frc::Translation2d camToTagTranslation{
      frc::Translation3d{
          bestTarget.GetBestCameraToTarget().Translation().Norm(),
          frc::Rotation3d{0_deg, -units::degree_t{bestTarget.GetPitch()},
                          -units::degree_t{bestTarget.GetYaw()}}}
          .RotateBy(m_robotToCamera.Rotation())
          .ToTranslation2d()
          .RotateBy(headingSample)};

  std::optional<frc::Pose3d> tagPose =
      aprilTags.GetTagPose(bestTarget.GetFiducialId());

  if (!tagPose.has_value()) {
    return {};
  }

  frc::Pose2d tagPose2d{tagPose->ToPose2d()};

  frc::Translation2d fieldToCameraTranslation{tagPose2d.Translation() +
                                              (-camToTagTranslation)};

  frc::Translation2d camToRobotTranslation{
      (-m_robotToCamera.Translation().ToTranslation2d())
          .RotateBy(headingSample)};

  frc::Pose2d robotPose{fieldToCameraTranslation + camToRobotTranslation,
                        headingSample};

  // fmt::print("Using trig distance\n");

  return EstimatedRobotPose{frc::Pose3d{robotPose}, result.GetTimestamp(),
                            result.GetTargets(),
                            PoseStrategy::PNP_DISTANCE_TRIG_SOLVE};
}

std::optional<str::EstimatedRobotPose> StrPoseEstimator::ConstrainedPnpStrategy(
    photon::PhotonPipelineResult result,
    std::optional<photon::PhotonCamera::CameraMatrix> camMat,
    std::optional<photon::PhotonCamera::DistortionMatrix> distCoeffs,
    std::optional<ConstrainedSolvepnpParams> constrainedPnpParams) {
  using namespace frc;

  if (!camMat || !distCoeffs) {
    FRC_ReportError(frc::warn::Warning,
                    "No camera calibration data provided to "
                    "StrPoseEstimator::MultiTagOnRioStrategy!",
                    "");
    return Update(result, this->multiTagFallbackStrategy);
  }

  if (!constrainedPnpParams) {
    return {};
  }

  if (!constrainedPnpParams->headingFree &&
      !headingBuffer.Sample(result.GetTimestamp()).has_value()) {
    return Update(result, camMat, distCoeffs, {},
                  this->multiTagFallbackStrategy);
  }

  frc::Pose3d fieldToRobotSeed;

  if (result.MultiTagResult().has_value()) {
    fieldToRobotSeed =
        frc::Pose3d{} + (result.MultiTagResult()->estimatedPose.best +
                         m_robotToCamera.Inverse());
  } else {
    std::optional<EstimatedRobotPose> nestedUpdate =
        Update(result, camMat, distCoeffs, {}, this->multiTagFallbackStrategy);

    if (!nestedUpdate.has_value()) {
      return {};
    }

    fieldToRobotSeed = nestedUpdate->estimatedPose;
  }

  if (!constrainedPnpParams.value().headingFree) {
    fieldToRobotSeed = frc::Pose3d{
        fieldToRobotSeed.Translation(),
        frc::Rotation3d{headingBuffer.Sample(result.GetTimestamp()).value()}};
  }

  std::vector<photon::PhotonTrackedTarget> targets{result.GetTargets().begin(),
                                                   result.GetTargets().end()};

  std::optional<photon::PnpResult> pnpResult =
      EstimateRobotPoseConstrainedSolvePNP(
          camMat.value(), distCoeffs.value(), targets, m_robotToCamera,
          fieldToRobotSeed, aprilTags, photon::kAprilTag36h11,
          constrainedPnpParams->headingFree,
          frc::Rotation2d{headingBuffer.Sample(result.GetTimestamp()).value()},
          constrainedPnpParams->headingScalingFactor);

  if (!pnpResult) {
    return Update(result, camMat, distCoeffs, {},
                  this->multiTagFallbackStrategy);
  }

  frc::Pose3d best = frc::Pose3d{} + pnpResult->best;

  // fmt::print("Using constrained solve pnp\n");

  return EstimatedRobotPose{best, result.GetTimestamp(), result.GetTargets(),
                            PoseStrategy::CONSTRAINED_SOLVEPNP};
}
}  // namespace str
