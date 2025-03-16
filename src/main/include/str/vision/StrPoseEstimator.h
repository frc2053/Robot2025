// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>

#include <memory>

#include <opencv2/core/mat.hpp>

#include "frc/geometry/Rotation2d.h"
#include "frc/interpolation/TimeInterpolatableBuffer.h"
#include "photon/PhotonCamera.h"
#include "photon/dataflow/structures/Packet.h"
#include "photon/targeting/MultiTargetPNPResult.h"
#include "photon/targeting/PhotonPipelineResult.h"
#include "photon/targeting/PhotonTrackedTarget.h"
#include "photon/targeting/PnpResult.h"
#include "units/angle.h"
#include "units/time.h"

namespace str {
enum PoseStrategy {
  LOWEST_AMBIGUITY = 0,
  CLOSEST_TO_CAMERA_HEIGHT,
  CLOSEST_TO_REFERENCE_POSE,
  CLOSEST_TO_LAST_POSE,
  AVERAGE_BEST_TARGETS,
  MULTI_TAG_PNP_ON_COPROCESSOR,
  MULTI_TAG_PNP_ON_RIO,
  CONSTRAINED_SOLVEPNP,
  PNP_DISTANCE_TRIG_SOLVE
};

struct ConstrainedSolvepnpParams {
  bool headingFree{false};
  double headingScalingFactor{0.0};
};

struct EstimatedRobotPose {
  /** The estimated pose */
  frc::Pose3d estimatedPose;
  /** The estimated time the frame used to derive the robot pose was taken, in
   * the same timebase as the RoboRIO FPGA Timestamp */
  units::second_t timestamp;

  /** A list of the targets used to compute this pose */
  wpi::SmallVector<photon::PhotonTrackedTarget, 10> targetsUsed;

  /** The strategy actually used to produce this pose */
  PoseStrategy strategy;

  EstimatedRobotPose(frc::Pose3d pose_, units::second_t time_,
                     std::span<const photon::PhotonTrackedTarget> targets,
                     PoseStrategy strategy_)
      : estimatedPose(pose_),
        timestamp(time_),
        targetsUsed(targets.data(), targets.data() + targets.size()),
        strategy(strategy_) {}
};

/**
 * The StrPoseEstimator class filters or combines readings from all the
 * fiducials visible at a given timestamp on the field to produce a single robot
 * in field pose, using the strategy set below. Example usage can be found in
 * our apriltagExample example project.
 */
class StrPoseEstimator {
 public:
  /**
   * Create a new StrPoseEstimator.
   *
   * @param aprilTags A AprilTagFieldLayout linking AprilTag IDs to Pose3ds with
   * respect to the FIRST field.
   * @param strategy The strategy it should use to determine the best pose.
   * @param robotToCamera Transform3d from the center of the robot to the camera
   * mount positions (ie, robot ➔ camera).
   */
  explicit StrPoseEstimator(frc::AprilTagFieldLayout aprilTags,
                            PoseStrategy strategy,
                            frc::Transform3d robotToCamera);

  /**
   * Get the AprilTagFieldLayout being used by the PositionEstimator.
   *
   * @return the AprilTagFieldLayout
   */
  frc::AprilTagFieldLayout GetFieldLayout() const { return aprilTags; }

  /**
   * Get the Position Estimation Strategy being used by the Position Estimator.
   *
   * @return the strategy
   */
  PoseStrategy GetPoseStrategy() const { return strategy; }

  /**
   * Set the Position Estimation Strategy used by the Position Estimator.
   *
   * @param strategy the strategy to set
   */
  inline void SetPoseStrategy(PoseStrategy strat) {
    if (strategy != strat) {
      InvalidatePoseCache();
    }
    strategy = strat;
  }

  /**
   * Set the Position Estimation Strategy used in multi-tag mode when
   * only one tag can be seen. Must NOT be MULTI_TAG_PNP
   *
   * @param strategy the strategy to set
   */
  void SetMultiTagFallbackStrategy(PoseStrategy strategy);

  /**
   * Return the reference position that is being used by the estimator.
   *
   * @return the referencePose
   */
  frc::Pose3d GetReferencePose() const { return referencePose; }

  /**
   * Update the stored reference pose for use when using the
   * CLOSEST_TO_REFERENCE_POSE strategy.
   *
   * @param referencePose the referencePose to set
   */
  inline void SetReferencePose(frc::Pose3d referencePose) {
    if (this->referencePose != referencePose) {
      InvalidatePoseCache();
    }
    this->referencePose = referencePose;
  }

  /**
   * @return The current transform from the center of the robot to the camera
   *         mount position.
   */
  inline frc::Transform3d GetRobotToCameraTransform() {
    return m_robotToCamera;
  }

  /**
   * Useful for pan and tilt mechanisms, or cameras on turrets
   *
   * @param robotToCamera The current transform from the center of the robot to
   * the camera mount position.
   */
  inline void SetRobotToCameraTransform(frc::Transform3d robotToCamera) {
    m_robotToCamera = robotToCamera;
  }

  /**
   * Update the stored last pose. Useful for setting the initial estimate when
   * using the CLOSEST_TO_LAST_POSE strategy.
   *
   * @param lastPose the lastPose to set
   */
  inline void SetLastPose(frc::Pose3d lastPose) { this->lastPose = lastPose; }

  /**
   * Update the pose estimator. If updating multiple times per loop, you should
   * call this exactly once per new result, in order of increasing result
   * timestamp.
   *
   * @param result The vision targeting result to process
   * @param cameraIntrinsics The camera calibration pinhole coefficients matrix.
   * Only required if doing multitag-on-rio, and may be nullopt otherwise.
   * @param distCoeffsData The camera calibration distortion coefficients. Only
   * required if doing multitag-on-rio, and may be nullopt otherwise.
   */
  std::optional<EstimatedRobotPose> Update(
      const photon::PhotonPipelineResult& result,
      std::optional<photon::PhotonCamera::CameraMatrix> cameraMatrixData =
          std::nullopt,
      std::optional<photon::PhotonCamera::DistortionMatrix> coeffsData =
          std::nullopt);

  std::optional<EstimatedRobotPose> Update(
      const photon::PhotonPipelineResult& result,
      std::optional<photon::PhotonCamera::CameraMatrix> cameraMatrixData,
      std::optional<photon::PhotonCamera::DistortionMatrix> coeffsData,
      std::optional<ConstrainedSolvepnpParams> constrainedPnpParams);

  void AddHeadingData(units::second_t timestamp,
                      const frc::Rotation2d& heading);

 private:
  frc::AprilTagFieldLayout aprilTags;
  PoseStrategy strategy;
  PoseStrategy multiTagFallbackStrategy = LOWEST_AMBIGUITY;

  frc::Transform3d m_robotToCamera;

  frc::Pose3d lastPose;
  frc::Pose3d referencePose;

  frc::TimeInterpolatableBuffer<units::radian_t> headingBuffer{1_s};

  units::second_t poseCacheTimestamp;

  inline static int InstanceCount = 0;

  inline void InvalidatePoseCache() { poseCacheTimestamp = -1_s; }

  /**
   * Internal convenience method for using a fallback strategy for update().
   * This should only be called after timestamp checks have been done by another
   * update() overload.
   *
   * @param cameraResult The latest pipeline result from the camera
   * @param strategy The pose strategy to use
   * @return an EstimatedRobotPose with an estimated pose, timestamp, and
   * targets used to create the estimate.
   */
  std::optional<EstimatedRobotPose> Update(
      const photon::PhotonPipelineResult& result, PoseStrategy strategy) {
    return Update(result, std::nullopt, std::nullopt, std::nullopt, strategy);
  }

  std::optional<EstimatedRobotPose> Update(
      const photon::PhotonPipelineResult& result,
      std::optional<photon::PhotonCamera::CameraMatrix> cameraMatrixData,
      std::optional<photon::PhotonCamera::DistortionMatrix> coeffsData,
      std::optional<ConstrainedSolvepnpParams> constrainedPnpParams,
      PoseStrategy strategy);

  /**
   * Return the estimated position of the robot with the lowest position
   * ambiguity from a List of pipeline results.
   *
   * @return the estimated position of the robot in the FCS and the estimated
   * timestamp of this estimation.
   */
  std::optional<EstimatedRobotPose> LowestAmbiguityStrategy(
      photon::PhotonPipelineResult result);

  /**
   * Return the estimated position of the robot using the target with the lowest
   * delta height difference between the estimated and actual height of the
   * camera.
   *
   * @return the estimated position of the robot in the FCS and the estimated
   * timestamp of this estimation.
   */
  std::optional<EstimatedRobotPose> ClosestToCameraHeightStrategy(
      photon::PhotonPipelineResult result);

  /**
   * Return the estimated position of the robot using the target with the lowest
   * delta in the vector magnitude between it and the reference pose.
   *
   * @param referencePose reference pose to check vector magnitude difference
   * against.
   * @return the estimated position of the robot in the FCS and the estimated
   * timestamp of this estimation.
   */
  std::optional<EstimatedRobotPose> ClosestToReferencePoseStrategy(
      photon::PhotonPipelineResult result);

  /**
   * Return the pose calculated by combining all tags into one on coprocessor
   *
   * @return the estimated position of the robot in the FCS
   */
  std::optional<EstimatedRobotPose> MultiTagOnCoprocStrategy(
      photon::PhotonPipelineResult result);

  /**
   * Return the pose calculation using all targets in view in the same PNP()
   calculation
   *
   * @return the estimated position of the robot in the FCS and the estimated
   timestamp of this estimation.
   */
  std::optional<EstimatedRobotPose> MultiTagOnRioStrategy(
      photon::PhotonPipelineResult result,
      std::optional<photon::PhotonCamera::CameraMatrix> camMat,
      std::optional<photon::PhotonCamera::DistortionMatrix> distCoeffs);

  /**
   * Return the average of the best target poses using ambiguity as weight.

   * @return the estimated position of the robot in the FCS and the estimated
   timestamp of this estimation.
   */
  std::optional<EstimatedRobotPose> AverageBestTargetsStrategy(
      photon::PhotonPipelineResult result);

  std::optional<EstimatedRobotPose> PnpDistanceTrigSolveStrategy(
      photon::PhotonPipelineResult result);

  std::optional<EstimatedRobotPose> ConstrainedPnpStrategy(
      photon::PhotonPipelineResult result,
      std::optional<photon::PhotonCamera::CameraMatrix> camMat,
      std::optional<photon::PhotonCamera::DistortionMatrix> distCoeffs,
      std::optional<ConstrainedSolvepnpParams> constrainedPnpParams);
};

}  // namespace str
