// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

#include <functional>
#include <memory>

#include "ctre/phoenix6/SignalLogger.hpp"
#include "frc/geometry/Pose2d.h"
#include "str/swerve/SwerveDrive.h"
#include "str/swerve/SwerveModuleHelpers.h"
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
  void SetupPathplanner();
  void AddVisionMeasurement(const frc::Pose2d& measurement,
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

  str::swerve::WheelRadiusCharData wheelRadiusData{};

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
