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

#include "constants/Constants.h"
#include "constants/SwerveConstants.h"
#include "frc/MathUtil.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Transform2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "frc2/command/button/Trigger.h"
#include "pathplanner/lib/util/DriveFeedforwards.h"
#include "pathplanner/lib/util/FlippingUtil.h"
#include "str/DriverstationUtils.h"
#include "str/swerve/SwerveModuleHelpers.h"
#include "units/angle.h"
#include "units/length.h"
#include "units/velocity.h"
#include "util/choreovariables.h"

Drive::Drive() {
  importantPoses = strchoreo::LoadPoses();
  SetupPathplanner();
  frc::SmartDashboard::PutNumber("L OFFSET",
                                 lOffset.convert<units::inches>().value());
  frc::SmartDashboard::PutNumber("R OFFSET",
                                 rOffset.convert<units::inches>().value());
  frc::SmartDashboard::PutNumber("GOTOPOSETRANSP",
                                 consts::swerve::pathplanning::RAW_POSE_P);
  frc::SmartDashboard::PutNumber("GOTOPOSETRANSD",
                                 consts::swerve::pathplanning::RAW_POSE_D);
  frc::SmartDashboard::PutNumber("GOTOPOSEROTP",
                                 consts::swerve::pathplanning::RAW_ROTATION_P);
  frc::SmartDashboard::PutNumber("GOTOPOSEROTD",
                                 consts::swerve::pathplanning::RAW_ROTATION_D);
}

void Drive::Periodic() {
  swerveDrive.UpdateNTEntries();
  SetPosePids();
  lOffset = units::inch_t{frc::SmartDashboard::GetNumber("L OFFSET", 0)};
  rOffset = units::inch_t{frc::SmartDashboard::GetNumber("R OFFSET", 0)};
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

frc2::Trigger Drive::IsAligned() {
  return frc2::Trigger{[this] { return isAtGoalState; }};
}

void Drive::SetPosePids() {
  double newTP = frc::SmartDashboard::GetNumber(
      "GOTOPOSETRANSP", consts::swerve::pathplanning::RAW_POSE_P);
  double newTD = frc::SmartDashboard::GetNumber(
      "GOTOPOSETRANSD", consts::swerve::pathplanning::RAW_POSE_D);
  double newRotP = frc::SmartDashboard::GetNumber(
      "GOTOPOSEROTP", consts::swerve::pathplanning::RAW_ROTATION_P);
  double newRotD = frc::SmartDashboard::GetNumber(
      "GOTOPOSEROTD", consts::swerve::pathplanning::RAW_ROTATION_D);
  translationController.SetPID(newTP, 0, newTD);
  thetaController.SetPID(newRotP, 0, newRotD);
}

units::meters_per_second_t Drive::CalculateSpeedAtGoal(
    frc::Translation2d currentTrans, frc::Translation2d goalTrans) {
  frc::ChassisSpeeds fieldVel = swerveDrive.GetFieldRelativeSpeeds();
  frc::Translation2d linearFieldVelocity{units::meter_t{fieldVel.vx.value()},
                                         units::meter_t{fieldVel.vy.value()}};
  return units::math::min(
      0.0_mps,
      units::meters_per_second_t{
          -linearFieldVelocity.RotateBy(-(goalTrans - currentTrans).Angle())
               .X()
               .value()});
}

frc2::CommandPtr Drive::DriveToPose(std::function<frc::Pose2d()> goalPose,
                                    bool useSingleTagEstimator) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce(
                 [this, goalPose, useSingleTagEstimator] {
                   // Which pose to use
                   frc::Pose2d currentPose =
                       useSingleTagEstimator ? swerveDrive.GetSingleTagPose()
                                             : GetRobotPose();

                   // Reset pid controllers
                   frc::ChassisSpeeds currentSpeeds =
                       swerveDrive.GetFieldRelativeSpeeds();
                   translationController.Reset(
                       currentPose.Translation().Distance(
                           goalPose().Translation()),
                       CalculateSpeedAtGoal(currentPose.Translation(),
                                            goalPose().Translation()));
                   thetaController.Reset(currentPose.Rotation().Radians(),
                                         currentSpeeds.omega);
                   thetaController.EnableContinuousInput(
                       units::radian_t{-std::numbers::pi},
                       units::radian_t{std::numbers::pi});
                   translationController.SetGoal(0_m);
                   thetaController.SetGoal(goalPose().Rotation().Radians());
                   translationController.SetTolerance(
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

                   frc::Pose2d target = goalPose();

                   frc::Translation2d currentPosition =
                       currentPose.Translation();
                   frc::Translation2d goalPosition = target.Translation();
                   frc::Translation2d vectorToTarget =
                       goalPosition - currentPosition;

                   units::meter_t distanceToTarget =
                       currentPosition.Distance(goalPosition);

                   double directionX = 0;
                   double directionY = 0;

                   if (distanceToTarget > 0.75_in) {
                     directionX =
                         vectorToTarget.X().value() / distanceToTarget.value();
                     directionY =
                         vectorToTarget.Y().value() / distanceToTarget.value();
                   }

                   units::meters_per_second_t translationSpeed{
                       translationController.Calculate(distanceToTarget, 0_m)};

                   translationSpeed = units::math::abs(translationSpeed);

                   units::meters_per_second_t xSpeed =
                       translationSpeed * directionX;
                   units::meters_per_second_t ySpeed =
                       translationSpeed * directionY;

                   units::radians_per_second_t thetaSpeed{
                       thetaController.Calculate(
                           currentPose.Rotation().Radians(),
                           target.Rotation().Radians())};

                   auto translationSetpoint =
                       translationController.GetSetpoint();
                   auto rotationSetpoint = thetaController.GetSetpoint();

                   frc::Translation2d setpointTranslation = currentPosition;
                   if (distanceToTarget > 0.01_m) {
                     double setpointDistance =
                         distanceToTarget.value() -
                         translationSetpoint.position.value();
                     setpointTranslation =
                         currentPosition +
                         frc::Translation2d{
                             units::meter_t{directionX * setpointDistance},
                             units::meter_t{directionY * setpointDistance}};
                   }

                   frc::Rotation2d setpointRotation{rotationSetpoint.position};
                   frc::Pose2d setpointPose{setpointTranslation,
                                            setpointRotation};

                   pidPoseGoalPub.Set(target);
                   pidPoseSetpointPub.Set(setpointPose);

                   pidPoseSpeeds.Set(frc::ChassisSpeeds{
                       xSpeed +
                           units::meters_per_second_t{
                               setpointTranslation.X().value()},
                       ySpeed, thetaSpeed});

                   swerveDrive.DriveFieldRelative(xSpeed, ySpeed, thetaSpeed,
                                                  false);
                 },
                 {this})
                 .Until([this] {
                   bool isAtGoal = translationController.AtGoal() &&
                                   thetaController.AtGoal();
                   isAtGoalState = isAtGoal;
                   isAtGoalPosePub.Set(isAtGoal);
                   return isAtGoal;
                 })
                 .WithName("PIDToPose Run"))
      .WithName("PIDToPose");
}

frc2::CommandPtr Drive::AlignToAlgae() {
  return DriveToPose(
      [this] {
        frc::Pose2d centerOfAlgae =
            importantPoses[WhatAlgaeToGoTo(WhatReefZoneAmIIn())];

        frc::Pose2d clawPos = centerOfAlgae;
        clawPos = clawPos.TransformBy(
            frc::Transform2d{0_m, lOffset, frc::Rotation2d{}});

        if (str::IsOnRed()) {
          return pathplanner::FlippingUtil::flipFieldPose(clawPos);
        } else {
          return clawPos;
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
        frc::Pose2d centerOfPole =
            importantPoses[WhatPoleToGoTo(WhatReefZoneAmIIn(), leftSide())];

        frc::Pose2d clawOnPole = centerOfPole;

        if (str::IsOnRed()) {
          if (leftSide()) {
            clawOnPole = clawOnPole.TransformBy(
                frc::Transform2d{0_m, rOffset, frc::Rotation2d{}});
          } else {
            clawOnPole = clawOnPole.TransformBy(
                frc::Transform2d{0_m, lOffset, frc::Rotation2d{}});
          }
          return pathplanner::FlippingUtil::flipFieldPose(clawOnPole);
        } else {
          if (leftSide()) {
            clawOnPole = clawOnPole.TransformBy(
                frc::Transform2d{0_m, lOffset, frc::Rotation2d{}});
          } else {
            clawOnPole = clawOnPole.TransformBy(
                frc::Transform2d{0_m, rOffset, frc::Rotation2d{}});
          }
          return clawOnPole;
        }
      },
      true);
}

frc2::CommandPtr Drive::AlignToReefSegment(std::function<bool()> leftSide,
                                           int zone) {
  return DriveToPose(
      [this, leftSide, zone] {
        frc::Pose2d centerOfPole =
            importantPoses[WhatPoleToGoTo(zone, leftSide())];

        frc::Pose2d clawOnPole = centerOfPole;

        if (str::IsOnRed()) {
          if (leftSide()) {
            clawOnPole = clawOnPole.TransformBy(
                frc::Transform2d{0_m, rOffset, frc::Rotation2d{}});
          } else {
            clawOnPole = clawOnPole.TransformBy(
                frc::Transform2d{0_m, lOffset, frc::Rotation2d{}});
          }
          return pathplanner::FlippingUtil::flipFieldPose(clawOnPole);
        } else {
          if (leftSide()) {
            clawOnPole = clawOnPole.TransformBy(
                frc::Transform2d{0_m, lOffset, frc::Rotation2d{}});
          } else {
            clawOnPole = clawOnPole.TransformBy(
                frc::Transform2d{0_m, rOffset, frc::Rotation2d{}});
          }
          return clawOnPole;
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

  units::radian_t angle =
      units::math::atan2(swerveDrive.GetPose().Y() - reefCenter.Y(),
                         swerveDrive.GetPose().X() - reefCenter.X());

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

frc2::CommandPtr Drive::SetDesiredTag(const std::string& newTag) {
  return frc2::cmd::RunOnce([this, newTag] { tagStr = newTag; }, {});
}

bool Drive::IsCloseToDesiredTag() {
  frc::Translation2d desiredTagTrans = importantPoses[tagStr].Translation();
  if (str::IsOnRed()) {
    desiredTagTrans =
        pathplanner::FlippingUtil::flipFieldPosition(desiredTagTrans);
  }

  return swerveDrive.GetPose().Translation().Distance(desiredTagTrans) <= 3_ft;
}

void Drive::SetupPathplanner() {
  consts::swerve::pathplanning::config =
      pathplanner::RobotConfig::fromGUISettings();
  ppControllers = std::make_shared<pathplanner::PPHolonomicDriveController>(
      pathplanner::PIDConstants{consts::swerve::pathplanning::POSE_P,
                                consts::swerve::pathplanning::POSE_I,
                                consts::swerve::pathplanning::POSE_D},
      pathplanner::PIDConstants{consts::swerve::pathplanning::ROTATION_P,
                                consts::swerve::pathplanning::ROTATION_I,
                                consts::swerve::pathplanning::ROTATION_D});

  pathplanner::AutoBuilder::configure(
      [this]() {
        if (IsCloseToDesiredTag()) {
          fmt::print("close enough, using single tag!\n");
          return swerveDrive.GetSingleTagPose();
        } else {
          return swerveDrive.GetPose();
        }
      },
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
        fmt::print("Active path changed!:\n");
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
