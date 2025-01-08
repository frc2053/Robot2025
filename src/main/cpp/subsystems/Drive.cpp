// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/Drive.h"

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

#include <memory>
#include <string>

#include "constants/SwerveConstants.h"
#include "frc/MathUtil.h"
#include "frc/geometry/Pose2d.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "pathplanner/lib/util/DriveFeedforwards.h"
#include "str/DriverstationUtils.h"
#include "str/swerve/SwerveModuleHelpers.h"

Drive::Drive() {
  SetupPathplanner();
}

void Drive::Periodic() {
  swerveDrive.UpdateNTEntries();
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
            str::swerve::SteerGains newGains{
                units::turns_per_second_t{frc::SmartDashboard::GetNumber(
                    tablePrefix + "mmCruiseVel", 0)},
                str::gains::radial::turn_volt_ka_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "mmKA", 0)},
                str::gains::radial::turn_volt_kv_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "mmKV", 0)},
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
