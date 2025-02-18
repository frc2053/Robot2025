// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/Drive.h"

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

#include <memory>
#include <numbers>
#include <string>

#include "constants/SwerveConstants.h"
#include "frc/MathUtil.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "pathplanner/lib/util/DriveFeedforwards.h"
#include "pathplanner/lib/util/FlippingUtil.h"
#include "str/DriverstationUtils.h"
#include "str/swerve/SwerveModuleHelpers.h"
#include "units/angle.h"
#include "util/choreovariables.h"

Drive::Drive() {
  importantPoses = strchoreo::LoadPoses();
  SetupPathplanner();
}

void Drive::Periodic() {
  swerveDrive.UpdateNTEntries();
  WhatReefZoneAmIIn();
}

void Drive::SimulationPeriodic() {
  swerveDrive.UpdateSimulation();
}

void Drive::UpdateOdom() {
  swerveDrive.UpdateOdom();
}

frc::Pose2d Drive::GetRobotPose() const {
  return swerveDrive.GetPose();
}

frc::Pose2d Drive::GetOdomPose() const {
  return swerveDrive.GetOdomPose();
}

void Drive::AddVisionMeasurement(const frc::Pose2d& measurement,
                                 units::second_t timestamp,
                                 const Eigen::Vector3d& stdDevs) {
  swerveDrive.AddVisionMeasurement(measurement, timestamp, stdDevs);
}

void Drive::AddSingleTagVisionMeasurement(const frc::Pose2d& measurement,
                                          units::second_t timestamp,
                                          const Eigen::Vector3d& stdDevs) {
  swerveDrive.AddSingleTagVisionMeasurement(measurement, timestamp, stdDevs);
}

frc2::CommandPtr Drive::DriveTeleop(
    std::function<units::meters_per_second_t()> xVel,
    std::function<units::meters_per_second_t()> yVel,
    std::function<units::radians_per_second_t()> omega) {
  return frc2::cmd::Run(
             [this, xVel, yVel, omega] {
               swerveDrive.DriveFieldRelative(xVel(), yVel(), omega(), true);
             },
             {this})
      .WithName("DriveTeleop");
}

frc2::CommandPtr Drive::DriveRobotRel(
    std::function<units::meters_per_second_t()> xVel,
    std::function<units::meters_per_second_t()> yVel,
    std::function<units::radians_per_second_t()> omega) {
  return frc2::cmd::Run(
             [this, xVel, yVel, omega] {
               swerveDrive.Drive(xVel(), yVel(), omega(), false);
             },
             {this})
      .WithName("DriveRobotRel");
}

frc2::CommandPtr Drive::DriveToPose(std::function<frc::Pose2d()> goalPose,
                                    bool useSingleTagEstimator) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce(
                 [this, goalPose, useSingleTagEstimator] {
                   frc::Pose2d currentPose =
                       useSingleTagEstimator ? swerveDrive.GetSingleTagPose()
                                             : GetRobotPose();
                   frc::ChassisSpeeds currentSpeeds =
                       swerveDrive.GetFieldRelativeSpeeds();
                   xPoseController.Reset(currentPose.Translation().X(),
                                         currentSpeeds.vx);
                   yPoseController.Reset(currentPose.Translation().Y(),
                                         currentSpeeds.vy);
                   thetaController.Reset(currentPose.Rotation().Radians(),
                                         currentSpeeds.omega);
                   thetaController.EnableContinuousInput(
                       units::radian_t{-std::numbers::pi},
                       units::radian_t{std::numbers::pi});
                   xPoseController.SetGoal(goalPose().X());
                   yPoseController.SetGoal(goalPose().Y());
                   thetaController.SetGoal(goalPose().Rotation().Radians());
                   xPoseController.SetTolerance(
                       consts::swerve::pathplanning::translationalPIDTolerance,
                       consts::swerve::pathplanning::
                           translationalVelPIDTolerance);
                   yPoseController.SetTolerance(
                       consts::swerve::pathplanning::translationalPIDTolerance,
                       consts::swerve::pathplanning::
                           translationalVelPIDTolerance);
                   thetaController.SetTolerance(
                       consts::swerve::pathplanning::rotationalPIDTolerance,
                       consts::swerve::pathplanning::rotationalVelPIDTolerance);
                   pidPoseGoalPub.Set(goalPose());
                 },
                 {this})
                 .WithName("PIDToPose Init"),
             frc2::cmd::Run(
                 [this, goalPose, useSingleTagEstimator] {
                   frc::Pose2d currentPose =
                       useSingleTagEstimator ? swerveDrive.GetSingleTagPose()
                                             : GetRobotPose();

                   xPoseController.SetGoal(goalPose().X());
                   yPoseController.SetGoal(goalPose().Y());
                   thetaController.SetGoal(goalPose().Rotation().Radians());
                   pidPoseGoalPub.Set(goalPose());
                   pidPoseSetpointPub.Set(
                       frc::Pose2d{xPoseController.GetSetpoint().position,
                                   yPoseController.GetSetpoint().position,
                                   thetaController.GetSetpoint().position});

                   units::meters_per_second_t xSpeed{xPoseController.Calculate(
                       currentPose.Translation().X())};
                   units::meters_per_second_t ySpeed{yPoseController.Calculate(
                       currentPose.Translation().Y())};
                   units::radians_per_second_t thetaSpeed{
                       thetaController.Calculate(
                           currentPose.Rotation().Radians())};

                   swerveDrive.DriveFieldRelative(xSpeed, ySpeed, thetaSpeed,
                                                  true);
                 },
                 {this})
                 .Until([this] {
                   bool isAtGoal = xPoseController.AtGoal() &&
                                   yPoseController.AtGoal() &&
                                   thetaController.AtGoal();
                   isAtGoalPosePub.Set(isAtGoal);
                   return isAtGoal;
                 })
                 .WithName("PIDToPose Run"),
             frc2::cmd::RunOnce([this] {
               swerveDrive.Drive(0_mps, 0_mps, 0_deg_per_s, false);
             }).WithName("PIDToPose Stop"))
      .WithName("PIDToPose");
}

frc2::CommandPtr Drive::AlignToAlgae() {
  return DriveToPose(
      [this] {
        if (str::IsOnRed()) {
          return pathplanner::FlippingUtil::flipFieldPose(
              importantPoses[WhatAlgaeToGoTo(WhatReefZoneAmIIn())]);

        } else {
          return importantPoses[WhatAlgaeToGoTo(WhatReefZoneAmIIn())];
        }
      },
      true);
}

frc2::CommandPtr Drive::AlignToProcessor() {
  return DriveToPose(
      [this] {
        if (str::IsOnRed()) {
          return pathplanner::FlippingUtil::flipFieldPose(
              importantPoses["Process"]);

        } else {
          return importantPoses["Process"];
        }
      },
      false);
}

frc2::CommandPtr Drive::AlignToReef(std::function<bool()> leftSide) {
  return DriveToPose(
      [this, leftSide] {
        if (str::IsOnRed()) {
          return pathplanner::FlippingUtil::flipFieldPose(
              importantPoses[WhatPoleToGoTo(WhatReefZoneAmIIn(), leftSide())]);

        } else {
          return importantPoses[WhatPoleToGoTo(WhatReefZoneAmIIn(),
                                               leftSide())];
        }
      },
      true);
}

std::string Drive::WhatPoleToGoTo(int zone, bool leftOrRight) {
  if (zone == 0) {
    return leftOrRight ? "H" : "G";
  }
  if (zone == 1) {
    return leftOrRight ? "J" : "I";
  }
  if (zone == 2) {
    return leftOrRight ? "K" : "L";
  }
  if (zone == 3) {
    return leftOrRight ? "A" : "B";
  }
  if (zone == 4) {
    return leftOrRight ? "C" : "D";
  }
  if (zone == 5) {
    return leftOrRight ? "F" : "E";
  }
  return "A";
}

std::string Drive::WhatAlgaeToGoTo(int zone) {
  if (zone == 0) {
    return "GHAlgae";
  }
  if (zone == 1) {
    return "IJAlgae";
  }
  if (zone == 2) {
    return "KLAlgae";
  }
  if (zone == 3) {
    return "ABAlgae";
  }
  if (zone == 4) {
    return "CDAlgae";
  }
  if (zone == 5) {
    return "EFAlgae";
  }
  return "A";
}

// 0 is the side closer to the middle of the field, CCW+ when viewed from the
// top
int Drive::WhatReefZoneAmIIn() {
  frc::Translation2d reefCenter{4.482401371002197_m, 4.037817478179932_m};
  units::radian_t rotationAmount = 0_deg;
  if (str::IsOnRed()) {
    reefCenter = pathplanner::FlippingUtil::flipFieldPosition(reefCenter);
    rotationAmount = 180_deg;
  }

  units::radian_t angle = units::math::atan2(
      GetRobotPose().Y() - reefCenter.Y(), GetRobotPose().X() - reefCenter.X());

  units::radian_t normalizedAngle =
      units::math::fmod(angle + units::radian_t{2 * std::numbers::pi},
                        units::radian_t{2 * std::numbers::pi});

  units::radian_t rotatedAngle = units::math::fmod(
      normalizedAngle + units::radian_t{std::numbers::pi / 6} + rotationAmount,
      units::radian_t{2 * std::numbers::pi});

  units::radian_t sliceWidth = units::radian_t{2 * std::numbers::pi} / 6.0;

  int sliceIndex = static_cast<int>(rotatedAngle / sliceWidth);

  return sliceIndex;
}

void Drive::SetupPathplanner() {
  ppControllers = std::make_shared<pathplanner::PPHolonomicDriveController>(
      pathplanner::PIDConstants{consts::swerve::pathplanning::POSE_P,
                                consts::swerve::pathplanning::POSE_I,
                                consts::swerve::pathplanning::POSE_D},
      pathplanner::PIDConstants{consts::swerve::pathplanning::ROTATION_P,
                                consts::swerve::pathplanning::ROTATION_I,
                                consts::swerve::pathplanning::ROTATION_D});

  pathplanner::AutoBuilder::configure(
      [this]() { return GetRobotPose(); },
      [this](frc::Pose2d pose) { swerveDrive.ResetPose(pose); },
      [this]() { return swerveDrive.GetRobotRelativeSpeeds(); },
      [this](frc::ChassisSpeeds speeds, pathplanner::DriveFeedforwards ff) {
        swerveDrive.Drive(speeds, false);
        swerveDrive.SetXModuleForces(ff.robotRelativeForcesX);
        swerveDrive.SetYModuleForces(ff.robotRelativeForcesY);
      },
      ppControllers, consts::swerve::pathplanning::config,
      []() { return str::IsOnRed(); }, this);

  pathplanner::PathPlannerLogging::setLogActivePathCallback(
      [this](std::vector<frc::Pose2d> poses) {
        swerveDrive.SetActivePath(poses);
      });
}

frc2::CommandPtr Drive::SysIdSteerQuasistaticVoltage(
    frc2::sysid::Direction dir) {
  return steerSysIdVoltage.Quasistatic(dir).WithName(
      "Steer Quasistatic Voltage");
}
frc2::CommandPtr Drive::SysIdSteerDynamicVoltage(frc2::sysid::Direction dir) {
  return steerSysIdVoltage.Dynamic(dir).WithName("Steer Dynamic Voltage");
}

frc2::CommandPtr Drive::SysIdSteerQuasistaticTorqueCurrent(
    frc2::sysid::Direction dir) {
  return steerSysIdTorqueCurrent.Quasistatic(dir).WithName(
      "Steer Quasistatic Torque Current");
}
frc2::CommandPtr Drive::SysIdSteerDynamicTorqueCurrent(
    frc2::sysid::Direction dir) {
  return steerSysIdTorqueCurrent.Dynamic(dir).WithName(
      "Steer Dynamic Torque Current");
}

frc2::CommandPtr Drive::SysIdDriveQuasistaticTorqueCurrent(
    frc2::sysid::Direction dir) {
  return driveSysid.Quasistatic(dir).WithName(
      "Drive Quasistatic Torque Current");
}
frc2::CommandPtr Drive::SysIdDriveDynamicTorqueCurrent(
    frc2::sysid::Direction dir) {
  return driveSysid.Dynamic(dir).WithName("Drive Dynamic Torque Current");
}

frc2::CommandPtr Drive::TuneSteerPID(std::function<bool()> isDone) {
  std::string tablePrefix = "SwerveDrive/steerGains/";
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce(
          [tablePrefix, this] {
            frc::SmartDashboard::PutNumber(tablePrefix + "setpoint", 0);
            frc::SmartDashboard::PutNumber(
                tablePrefix + "mmCruiseVel",
                consts::swerve::gains::STEER.motionMagicCruiseVel.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "mmKA",
                consts::swerve::gains::STEER.motionMagicExpoKa.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "mmKV",
                consts::swerve::gains::STEER.motionMagicExpoKv.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kA", consts::swerve::gains::STEER.kA.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kV", consts::swerve::gains::STEER.kV.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kS", consts::swerve::gains::STEER.kS.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kP", consts::swerve::gains::STEER.kP.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kI", consts::swerve::gains::STEER.kI.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kD", consts::swerve::gains::STEER.kD.value());
            frc::SwerveModuleState zeroState{0_mps, frc::Rotation2d{0_rad}};
            swerveDrive.SetModuleStates(
                {zeroState, zeroState, zeroState, zeroState}, true, true, {});
          },
          {this}),
      frc2::cmd::Run(
          [this, tablePrefix] {
            str::gains::radial::VoltRadialGainsHolder newGains{
                units::turns_per_second_t{frc::SmartDashboard::GetNumber(
                    tablePrefix + "mmCruiseVel", 0)},
                str::gains::radial::turn_volt_ka_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "mmKA", 0)},
                str::gains::radial::turn_volt_kv_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "mmKV", 0)},
                str::gains::radial::turn_volt_ka_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kA", 0)},
                str::gains::radial::turn_volt_kv_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kV", 0)},
                units::volt_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kS", 0)},
                str::gains::radial::turn_volt_kp_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kP", 0)},
                str::gains::radial::turn_volt_ki_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kI", 0)},
                str::gains::radial::turn_volt_kd_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kD", 0)}};

            if (newGains != swerveDrive.GetSteerGains()) {
              for (int i = 0; i < 4; i++) {
                swerveDrive.SetSteerGains(newGains);
              }
            }

            for (int i = 0; i < 4; i++) {
              frc::SwerveModuleState state{
                  0_mps, frc::Rotation2d{
                             units::degree_t{frc::SmartDashboard::GetNumber(
                                 tablePrefix + "setpoint", 0)}}};
              swerveDrive.SetModuleStates({state, state, state, state}, true,
                                          true, {});
            }
          },
          {this})
          .Until(isDone));
}

frc2::CommandPtr Drive::TuneDrivePID(std::function<bool()> isDone) {
  std::string tablePrefix = "SwerveDrive/driveGains/";
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce(
          [tablePrefix, this] {
            frc::SmartDashboard::PutNumber(tablePrefix + "setpoint", 0);
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kA", consts::swerve::gains::DRIVE.kA.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kV", consts::swerve::gains::DRIVE.kV.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kS", consts::swerve::gains::DRIVE.kS.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kP", consts::swerve::gains::DRIVE.kP.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kI", consts::swerve::gains::DRIVE.kI.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kD", consts::swerve::gains::DRIVE.kD.value());
            frc::SwerveModuleState zeroState{0_mps, frc::Rotation2d{0_rad}};
            swerveDrive.SetModuleStates(
                {zeroState, zeroState, zeroState, zeroState}, true, true, {});
          },
          {this}),
      frc2::cmd::Run(
          [this, tablePrefix] {
            str::swerve::DriveGains newGains{
                str::gains::radial::turn_amp_ka_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kA", 0)},
                str::gains::radial::turn_amp_kv_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kV", 0)},
                units::ampere_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kS", 0)},
                str::gains::radial::turn_amp_kp_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kP", 0)},
                str::gains::radial::turn_amp_ki_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kI", 0)},
                str::gains::radial::turn_amp_kd_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kD", 0)}};

            if (newGains != swerveDrive.GetDriveGains()) {
              for (int i = 0; i < 4; i++) {
                swerveDrive.SetDriveGains(newGains);
              }
            }

            for (int i = 0; i < 4; i++) {
              frc::SwerveModuleState state{
                  units::feet_per_second_t{frc::SmartDashboard::GetNumber(
                      tablePrefix + "setpoint", 0)},
                  frc::Rotation2d{0_deg}};
              swerveDrive.SetModuleStates({state, state, state, state}, true,
                                          false, {});
            }
          },
          {this})
          .Until(isDone));
}

frc2::CommandPtr Drive::WheelRadius(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce(
                 [this] {
                   wheelRadiusData.lastGyroYaw = swerveDrive.GetYawFromImu();
                   wheelRadiusData.accumGyroYaw = 0_rad;
                   wheelRadiusData.startWheelPositions =
                       swerveDrive.GetModuleDriveOutputShaftPositions();
                   wheelRadiusData.omegaLimiter.Reset(0_rad_per_s);
                   wheelRadiusData.effectiveWheelRadius = 0_in;
                 },
                 {this}),
             frc2::cmd::RunEnd(
                 [this, dir] {
                   double dirMulti = 1.0;
                   if (dir == frc2::sysid::Direction::kReverse) {
                     dirMulti = -1.0;
                   }
                   units::radian_t currentYaw = swerveDrive.GetYawFromImu();
                   swerveDrive.Drive(0_mps, 0_mps,
                                     wheelRadiusData.omegaLimiter.Calculate(
                                         1_rad_per_s * dirMulti),
                                     true);
                   wheelRadiusData.accumGyroYaw += frc::AngleModulus(
                       currentYaw - wheelRadiusData.lastGyroYaw);
                   wheelRadiusData.lastGyroYaw = currentYaw;
                   units::radian_t avgWheelPos = 0.0_rad;
                   std::array<units::radian_t, 4> currentPositions;
                   currentPositions =
                       swerveDrive.GetModuleDriveOutputShaftPositions();
                   for (int i = 0; i < 4; i++) {
                     avgWheelPos += units::math::abs(
                         currentPositions[i] -
                         wheelRadiusData.startWheelPositions[i]);
                   }
                   avgWheelPos /= 4.0;
                   wheelRadiusData.effectiveWheelRadius =
                       (wheelRadiusData.accumGyroYaw *
                        consts::swerve::physical::DRIVEBASE_RADIUS) /
                       avgWheelPos;
                 },
                 [this] {
                   swerveDrive.Drive(0_mps, 0_mps, 0_rad_per_s, true);
                   frc::DataLogManager::Log(
                       fmt::format("WHEEL RADIUS: {}\n\n\n\n\n",
                                   wheelRadiusData.effectiveWheelRadius
                                       .convert<units::inches>()
                                       .value()));
                 },
                 {this}))
      .WithName("Wheel Radius Calculation");
}
