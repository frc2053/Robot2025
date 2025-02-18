// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "constants/SwerveConstants.h"
#include "ctre/phoenix6/SignalLogger.hpp"
#include "frc/geometry/Pose2d.h"
#include "frc2/command/CommandPtr.h"
#include "networktables/BooleanTopic.h"
#include "str/swerve/SwerveDrive.h"
#include "str/swerve/SwerveModuleHelpers.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/time.h"
#include "units/velocity.h"

class Drive : public frc2::SubsystemBase {
 public:
  Drive();
  void Periodic() override;
  void SimulationPeriodic() override;
  void UpdateOdom();
  frc::Pose2d GetRobotPose() const;
  frc::Pose2d GetOdomPose() const;
  units::radian_t GetGyroYaw() const { return swerveDrive.GetYawFromImu(); }
  void SetupPathplanner();
  void AddVisionMeasurement(const frc::Pose2d& measurement,
                            units::second_t timestamp,
                            const Eigen::Vector3d& stdDevs);
  void AddSingleTagVisionMeasurement(const frc::Pose2d& measurement,
                                     units::second_t timestamp,
                                     const Eigen::Vector3d& stdDevs);

  frc2::CommandPtr DriveTeleop(
      std::function<units::meters_per_second_t()> xVel,
      std::function<units::meters_per_second_t()> yVel,
      std::function<units::radians_per_second_t()> omega);

  frc2::CommandPtr DriveRobotRel(
      std::function<units::meters_per_second_t()> xVel,
      std::function<units::meters_per_second_t()> yVel,
      std::function<units::radians_per_second_t()> omega);

  frc2::CommandPtr AlignToReef(std::function<bool()> leftSide);
  frc2::CommandPtr AlignToAlgae();
  frc2::CommandPtr AlignToProcessor();
  frc2::CommandPtr DriveToPose(std::function<frc::Pose2d()> goalPose,
                               bool useSingleTagEstimator);

  frc2::CommandPtr SysIdSteerQuasistaticVoltage(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdSteerDynamicVoltage(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdSteerQuasistaticTorqueCurrent(
      frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdSteerDynamicTorqueCurrent(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdDriveQuasistaticTorqueCurrent(
      frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdDriveDynamicTorqueCurrent(frc2::sysid::Direction dir);
  frc2::CommandPtr TuneSteerPID(std::function<bool()> isDone);
  frc2::CommandPtr TuneDrivePID(std::function<bool()> isDone);
  frc2::CommandPtr WheelRadius(frc2::sysid::Direction dir);

 private:
  str::swerve::SwerveDrive swerveDrive{};
  std::shared_ptr<pathplanner::PPHolonomicDriveController> ppControllers;

  frc::TrapezoidProfile<units::meters>::Constraints translationConstraints{
      consts::swerve::physical::DRIVE_MAX_SPEED,
      consts::swerve::physical::MAX_ACCEL,
  };

  frc::TrapezoidProfile<units::radians>::Constraints rotationConstraints{
      consts::swerve::physical::MAX_ROT_SPEED,
      consts::swerve::physical::MAX_ROT_ACCEL,
  };

  frc::ProfiledPIDController<units::meters> xPoseController{
      consts::swerve::pathplanning::POSE_P,
      consts::swerve::pathplanning::POSE_I,
      consts::swerve::pathplanning::POSE_D, translationConstraints};

  frc::ProfiledPIDController<units::meters> yPoseController{
      consts::swerve::pathplanning::POSE_P,
      consts::swerve::pathplanning::POSE_I,
      consts::swerve::pathplanning::POSE_D, translationConstraints};

  frc::ProfiledPIDController<units::radians> thetaController{
      consts::swerve::pathplanning::ROTATION_P,
      consts::swerve::pathplanning::ROTATION_I,
      consts::swerve::pathplanning::ROTATION_D, rotationConstraints};

  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("Swerve")};
  nt::StructPublisher<frc::Pose2d> pidPoseGoalPub{
      nt->GetStructTopic<frc::Pose2d>("PIDToPoseGoal").Publish()};
  nt::StructPublisher<frc::Pose2d> pidPoseSetpointPub{
      nt->GetStructTopic<frc::Pose2d>("PIDToPoseSetpoint").Publish()};
  nt::BooleanPublisher isAtGoalPosePub{
      nt->GetBooleanTopic("PIDToPoseIsAtGoal").Publish()};

  str::swerve::WheelRadiusCharData wheelRadiusData{};

  std::unordered_map<std::string, frc::Pose2d> importantPoses{};
  int WhatReefZoneAmIIn();
  std::string WhatPoleToGoTo(int zone, bool leftOrRight);
  std::string WhatAlgaeToGoTo(int zone);

  frc2::sysid::SysIdRoutine steerSysIdVoltage{
      frc2::sysid::Config{
          std::nullopt, 7_V, std::nullopt,
          [](frc::sysid::State state) {
            ctre::phoenix6::SignalLogger().WriteString(
                "SysIdSteer_State",
                frc::sysid::SysIdRoutineLog::StateEnumToString(state));
          }},
      frc2::sysid::Mechanism{[this](units::volt_t voltsToSend) {
                               swerveDrive.SetCharacterizationVoltsSteer(
                                   voltsToSend);
                             },
                             [this](frc::sysid::SysIdRoutineLog* log) {
                               swerveDrive.LogSteerVolts(log);
                             },
                             this, "swerve-steer"}};

  frc2::sysid::SysIdRoutine steerSysIdTorqueCurrent{
      frc2::sysid::Config{
          (2_V / 1_s), 20_V, std::nullopt,
          [](frc::sysid::State state) {
            ctre::phoenix6::SignalLogger().WriteString(
                "SysIdSteer_State",
                frc::sysid::SysIdRoutineLog::StateEnumToString(state));
          }},
      frc2::sysid::Mechanism{[this](units::volt_t ampsToSend) {
                               swerveDrive.SetCharacterizationAmpsSteer(
                                   units::ampere_t{ampsToSend.value()});
                             },
                             [this](frc::sysid::SysIdRoutineLog* log) {
                               swerveDrive.LogSteerTorqueCurrent(log);
                             },
                             this, "swerve-steer"}};

  frc2::sysid::SysIdRoutine driveSysid{
      frc2::sysid::Config{
          (6_V / 1_s), 27_V, std::nullopt,
          [](frc::sysid::State state) {
            ctre::phoenix6::SignalLogger().WriteString(
                "SysIdDrive_State",
                frc::sysid::SysIdRoutineLog::StateEnumToString(state));
          }},
      frc2::sysid::Mechanism{[this](units::volt_t ampsToSend) {
                               swerveDrive.SetCharacterizationAmpsDrive(
                                   units::ampere_t{ampsToSend.value()});
                             },
                             [this](frc::sysid::SysIdRoutineLog* log) {
                               swerveDrive.LogDriveTorqueCurrent(log);
                             },
                             this, "swerve-drive"}};
};
