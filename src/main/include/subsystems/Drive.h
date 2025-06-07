// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#include "constants/Constants.h"
#include "constants/SwerveConstants.h"
#include "ctre/phoenix6/SignalLogger.hpp"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/button/Trigger.h"
#include "networktables/BooleanTopic.h"
#include "str/swerve/SwerveDrive.h"
#include "str/swerve/SwerveModuleHelpers.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/length.h"
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
  units::radian_t GetGyroYaw() const {
    return swerveDrive.GetOdomPose().Rotation().Radians();
  }
  std::string tagStr = "Tag20";
  frc2::CommandPtr OhShitButton();
  frc2::Trigger IsAligned();
  frc2::CommandPtr SetDesiredTag(const std::string& newTag);
  bool IsCloseToDesiredTag();
  void SetupPathplanner();
  frc2::CommandPtr LockWheels(std::function<bool()> override);
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

  frc2::CommandPtr AlignToReefSegment(std::function<bool()> leftSide, int zone);
  frc2::CommandPtr AlignToReef(std::function<bool()> leftSide);
  frc2::CommandPtr AlignToAlgae(std::function<bool()> shouldSpin);
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
  void SetPosePids();

 private:
  str::swerve::SwerveDrive swerveDrive{};
  std::shared_ptr<pathplanner::PPHolonomicDriveController> ppControllers;

  frc::TrapezoidProfile<units::meters>::Constraints translationConstraints{
      consts::swerve::physical::DRIVE_MAX_SPEED,
      consts::swerve::physical::MAX_ACCEL,
  };

  frc::TrapezoidProfile<units::radians>::Constraints rotationConstraints{
      consts::swerve::physical::MAX_ROT_SPEED,
      consts::swerve::physical::MAX_ROT_ACCEL * .5,
  };

  units::meters_per_second_t CalculateSpeedAtGoal(
      frc::Translation2d currentTrans, frc::Translation2d goalTrans);

  frc::ProfiledPIDController<units::meters> translationController{
      consts::swerve::pathplanning::RAW_POSE_P,
      consts::swerve::pathplanning::RAW_POSE_I,
      consts::swerve::pathplanning::RAW_POSE_D, translationConstraints};

  frc::ProfiledPIDController<units::radians> thetaController{
      consts::swerve::pathplanning::RAW_ROTATION_P,
      consts::swerve::pathplanning::RAW_ROTATION_I,
      consts::swerve::pathplanning::RAW_ROTATION_D, rotationConstraints};

  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("Swerve")};
  nt::StructPublisher<frc::Pose2d> pidPoseGoalPub{
      nt->GetStructTopic<frc::Pose2d>("PIDToPoseGoal").Publish()};
  nt::StructPublisher<frc::Pose2d> pidPoseSetpointPub{
      nt->GetStructTopic<frc::Pose2d>("PIDToPoseSetpoint").Publish()};
  nt::BooleanPublisher isAtGoalPosePub{
      nt->GetBooleanTopic("PIDToPoseIsAtGoal").Publish()};
  nt::StructPublisher<frc::ChassisSpeeds> pidPoseSpeeds{
      nt->GetStructTopic<frc::ChassisSpeeds>("PIDToPoseSpeeds").Publish()};

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
          std::nullopt, std::nullopt, std::nullopt,
          [](frc::sysid::State state) {
            ctre::phoenix6::SignalLogger().WriteString(
                "SysIdDrive_State",
                frc::sysid::SysIdRoutineLog::StateEnumToString(state));
          }},
      frc2::sysid::Mechanism{[this](units::volt_t voltsToSend) {
                               swerveDrive.SetCharacterizationVoltsDrive(
                                   voltsToSend);
                             },
                             [this](frc::sysid::SysIdRoutineLog* log) {
                               swerveDrive.LogDriveVolts(log);
                             },
                             this, "swerve-drive"}};

  units::meter_t lOffset{consts::yearspecific::CLAW_OFFSET_L};
  units::meter_t rOffset{consts::yearspecific::CLAW_OFFSET_R};

  bool isAtGoalState;
};
