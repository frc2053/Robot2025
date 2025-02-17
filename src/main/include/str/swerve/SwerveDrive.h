// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/sysid/SysIdRoutineLog.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>

#include <memory>
#include <vector>

#include <ctre/phoenix6/Pigeon2.hpp>

#include "constants/SwerveConstants.h"
#include "frc/Alert.h"
#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "str/swerve/SwerveModule.h"
#include "str/swerve/SwerveModuleHelpers.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/velocity.h"

namespace str::swerve {
class SwerveDrive {
 public:
  SwerveDrive();
  frc::Pose2d GetPose() const;
  frc::Pose2d GetOdomPose() const;
  frc::Pose2d GetSingleTagPose() const;

  void SetXModuleForces(const std::vector<units::newton_t>& xForce);
  void SetYModuleForces(const std::vector<units::newton_t>& yForce);
  void UpdateOdom();
  void UpdateSimulation();
  void UpdateNTEntries();
  void AddVisionMeasurement(const frc::Pose2d& measurement,
                            units::second_t timestamp,
                            const Eigen::Vector3d& stdDevs);
  void AddSingleTagVisionMeasurement(const frc::Pose2d& measurement,
                                     units::second_t timestamp,
                                     const Eigen::Vector3d& stdDevs);
  void DriveFieldRelative(units::meters_per_second_t xVel,
                          units::meters_per_second_t yVel,
                          units::radians_per_second_t omega, bool openLoop);
  frc::ChassisSpeeds GetRobotRelativeSpeeds();
  frc::ChassisSpeeds GetFieldRelativeSpeeds();
  void Drive(frc::ChassisSpeeds speeds, bool openLoop);
  void Drive(units::meters_per_second_t xVel, units::meters_per_second_t yVel,
             units::radians_per_second_t omega, bool openLoop);

  units::radian_t GetYawFromImu() const;
  void ZeroYaw();
  void ResetPose(const frc::Pose2d& resetPose);
  str::gains::radial::VoltRadialGainsHolder GetSteerGains() const;
  void SetSteerGains(str::gains::radial::VoltRadialGainsHolder newGains);
  str::swerve::DriveGains GetDriveGains() const;
  void SetDriveGains(str::swerve::DriveGains newGains);

  void SetCharacterizationVoltsSteer(units::volt_t volts);
  void SetCharacterizationAmpsSteer(units::ampere_t amps);
  void SetCharacterizationAmpsDrive(units::ampere_t amps);
  void LogSteerVolts(frc::sysid::SysIdRoutineLog* log);
  void LogSteerTorqueCurrent(frc::sysid::SysIdRoutineLog* log);
  void LogDriveTorqueCurrent(frc::sysid::SysIdRoutineLog* log);
  void SetModuleStates(
      const std::array<frc::SwerveModuleState, 4>& desiredStates, bool optimize,
      bool openLoop,
      const std::array<units::ampere_t, 4> moduleTorqueCurrentFF);
  std::array<units::radian_t, 4> GetModuleDriveOutputShaftPositions();
  void SetActivePath(std::vector<frc::Pose2d> poses);

 private:
  void SetupSignals();
  void ConfigureImu();
  std::array<units::ampere_t, 4> ConvertModuleForcesToTorqueCurrent(
      const std::vector<units::newton_t>& xForce,
      const std::vector<units::newton_t>& yForce);

  std::array<SwerveModule, 4> modules{
      SwerveModule{consts::swerve::physical::FL,
                   consts::swerve::physical::PHY_CHAR,
                   consts::swerve::gains::STEER, consts::swerve::gains::DRIVE},
      SwerveModule{consts::swerve::physical::FR,
                   consts::swerve::physical::PHY_CHAR,
                   consts::swerve::gains::STEER, consts::swerve::gains::DRIVE},
      SwerveModule{consts::swerve::physical::BL,
                   consts::swerve::physical::PHY_CHAR,
                   consts::swerve::gains::STEER, consts::swerve::gains::DRIVE},
      SwerveModule{consts::swerve::physical::BR,
                   consts::swerve::physical::PHY_CHAR,
                   consts::swerve::gains::STEER, consts::swerve::gains::DRIVE}};

  ctre::phoenix6::hardware::Pigeon2 imu{consts::swerve::can_ids::IMU, "*"};
  ctre::phoenix6::sim::Pigeon2SimState& imuSimState = imu.GetSimState();

  std::array<ctre::phoenix6::BaseStatusSignal*, 34> allSignals;

  std::array<frc::SwerveModulePosition, 4> modulePositions;
  std::array<frc::SwerveModuleState, 4> moduleStates;
  units::radian_t yawLatencyComped{0_rad};
  units::second_t lastOdomUpdateTime{0_s};
  units::hertz_t odomUpdateRate{0_Hz};
  frc::Rotation2d lastSimAngle;
  std::vector<units::newton_t> xModuleForce{};
  std::vector<units::newton_t> yModuleForce{};

  frc::SwerveDriveOdometry<4> odom{consts::swerve::physical::KINEMATICS,
                                   frc::Rotation2d{0_deg}, modulePositions};
  frc::SwerveDrivePoseEstimator<4> poseEstimator{
      consts::swerve::physical::KINEMATICS, frc::Rotation2d{0_deg},
      modulePositions, frc::Pose2d{}};
  frc::SwerveDrivePoseEstimator<4> singleTagPoseEstimator{
      consts::swerve::physical::KINEMATICS, frc::Rotation2d{0_deg},
      modulePositions, frc::Pose2d{}};

  static constexpr std::string_view imuConfigAlertStr = "Imu Configuration";
  static constexpr std::string_view imuOptimizeAlertStr = "Imu Optimization";
  static constexpr std::string_view imuZeroAlertStr = "Imu Zeroing";
  static constexpr std::string_view updateFreqAlertStr =
      "Swerve Signal Update Frequency";

  frc::Alert imuConfigAlert;
  frc::Alert imuOptimizeAlert;
  frc::Alert imuZeroAlert;
  frc::Alert updateFreqAlert;

  frc::Field2d swerveField{};
  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("Swerve")};
  nt::StructArrayPublisher<frc::SwerveModuleState> simStatesPub{
      nt->GetStructArrayTopic<frc::SwerveModuleState>("SimStates").Publish()};
  nt::StructPublisher<frc::ChassisSpeeds> simChassiSpeeds{
      nt->GetStructTopic<frc::ChassisSpeeds>("SimChassisSpeeds").Publish()};
  nt::StructArrayPublisher<frc::SwerveModuleState> desiredStatesPub{
      nt->GetStructArrayTopic<frc::SwerveModuleState>("DesiredStates")
          .Publish()};
  nt::StructArrayPublisher<frc::SwerveModuleState> currentStatesPub{
      nt->GetStructArrayTopic<frc::SwerveModuleState>("CurrentStates")
          .Publish()};
  nt::StructArrayPublisher<frc::Pose2d> activePathPub{
      nt->GetStructArrayTopic<frc::Pose2d>("ActivePath").Publish()};
  nt::StructArrayPublisher<frc::SwerveModulePosition> currentPositionsPub{
      nt->GetStructArrayTopic<frc::SwerveModulePosition>("CurrentPositions")
          .Publish()};
  nt::StructArrayPublisher<frc::SwerveModuleState> forcesPub{
      nt->GetStructArrayTopic<frc::SwerveModuleState>("PathForcesPub")
          .Publish()};
  nt::StructPublisher<frc::Pose2d> odomPosePub{
      nt->GetStructTopic<frc::Pose2d>("OdometryPose").Publish()};
  nt::StructPublisher<frc::Pose2d> addedVisionPosesPub{
      nt->GetStructTopic<frc::Pose2d>("AddedVisionPoses").Publish()};
  nt::StructPublisher<frc::Pose2d> estimatorPub{
      nt->GetStructTopic<frc::Pose2d>("PoseEstimatorPose").Publish()};
  nt::StructPublisher<frc::Pose2d> addedSingleTagVisionPosesPub{
      nt->GetStructTopic<frc::Pose2d>("AddedSingleTagVisionPoses").Publish()};
  nt::StructPublisher<frc::Pose2d> singleTagEstimatorPub{
      nt->GetStructTopic<frc::Pose2d>("SingleTagPoseEstimatorPose").Publish()};
  nt::DoublePublisher odomUpdateRatePub{
      nt->GetDoubleTopic("OdomUpdateRate").Publish()};
};
}  // namespace str::swerve
