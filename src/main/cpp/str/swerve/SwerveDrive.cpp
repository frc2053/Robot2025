// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/swerve/SwerveDrive.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>

#include <vector>

#include "constants/SwerveConstants.h"
#include "ctre/phoenix/StatusCodes.h"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "frc/Alert.h"
#include "frc/RobotController.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "str/DriverstationUtils.h"
#include "str/Math.h"
#include "str/swerve/SwerveModuleHelpers.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/voltage.h"

using namespace str::swerve;

SwerveDrive::SwerveDrive()
    : imuConfigAlert{imuConfigAlertStr, frc::Alert::AlertType::kError},
      imuOptimizeAlert{imuOptimizeAlertStr, frc::Alert::AlertType::kError},
      imuZeroAlert(imuZeroAlertStr, frc::Alert::AlertType::kError),
      updateFreqAlert(updateFreqAlertStr, frc::Alert::AlertType::kError) {
  ConfigureImu();
  SetupSignals();
  frc::SmartDashboard::PutData("SwerveField", &swerveField);
}

frc::ChassisSpeeds SwerveDrive::GetRobotRelativeSpeeds() {
  return consts::swerve::physical::KINEMATICS.ToChassisSpeeds(moduleStates);
}

frc::ChassisSpeeds SwerveDrive::GetFieldRelativeSpeeds() {
  return frc::ChassisSpeeds::FromRobotRelativeSpeeds(GetRobotRelativeSpeeds(),
                                                     GetPose().Rotation());
}

frc::Pose2d SwerveDrive::GetPose() const {
  return poseEstimator.GetEstimatedPosition();
}

frc::Pose2d SwerveDrive::GetOdomPose() const {
  return odom.GetPose();
}

frc::Pose2d SwerveDrive::GetSingleTagPose() const {
  return singleTagPoseEstimator.GetEstimatedPosition();
}

void SwerveDrive::SetXModuleForces(const std::vector<units::newton_t>& xForce) {
  xModuleForce = xForce;
}

void SwerveDrive::SetYModuleForces(const std::vector<units::newton_t>& yForce) {
  yModuleForce = yForce;
}

void SwerveDrive::SetActivePath(std::vector<frc::Pose2d> poses) {
  activePathPub.Set(poses);
  swerveField.GetObject("activePath")->SetPoses(poses);
}

void SwerveDrive::UpdateOdom() {
  ctre::phoenix::StatusCode status =
      ctre::phoenix6::BaseStatusSignal::WaitForAll(
          2.0 / consts::swerve::ODOM_UPDATE_RATE, allSignals);

  if (!status.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "Error updating swerve odom! Error was: {}", status.GetName()));
  }

  int i = 0;
  for (auto& mod : modules) {
    modulePositions[i] = mod.GetPosition();
    moduleStates[i] = mod.GetState();
    i++;
  }

  yawLatencyComped =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          imu.GetYaw(), imu.GetAngularVelocityZWorld());
  poseEstimator.Update(frc::Rotation2d{yawLatencyComped}, modulePositions);
  singleTagPoseEstimator.Update(frc::Rotation2d{yawLatencyComped},
                                modulePositions);
  odom.Update(frc::Rotation2d{yawLatencyComped}, modulePositions);

  units::second_t now = frc::Timer::GetFPGATimestamp();
  odomUpdateRate = 1.0 / (now - lastOdomUpdateTime);
  lastOdomUpdateTime = now;
}

void SwerveDrive::AddVisionMeasurement(const frc::Pose2d& measurement,
                                       units::second_t timestamp,
                                       const Eigen::Vector3d& stdDevs) {
  if (str::math::IsRobotInsideField(consts::swerve::physical::TOTAL_LENGTH,
                                    consts::swerve::physical::TOTAL_WIDTH,
                                    measurement)) {
    wpi::array<double, 3> newStdDevs{stdDevs(0), stdDevs(1), stdDevs(2)};
    addedVisionPosesPub.Set(measurement);
    poseEstimator.AddVisionMeasurement(measurement, timestamp, newStdDevs);
  } else {
    // frc::DataLogManager::Log(
    //     "WARNING: Vision pose was outside of field! Not adding to
    //     estimator!");
  }
}

void SwerveDrive::AddSingleTagVisionMeasurement(
    const frc::Pose2d& measurement, units::second_t timestamp,
    const Eigen::Vector3d& stdDevs) {
  if (str::math::IsRobotInsideField(consts::swerve::physical::TOTAL_LENGTH,
                                    consts::swerve::physical::TOTAL_WIDTH,
                                    measurement)) {
    wpi::array<double, 3> newStdDevs{stdDevs(0), stdDevs(1), stdDevs(2)};
    addedSingleTagVisionPosesPub.Set(measurement);
    singleTagPoseEstimator.AddVisionMeasurement(measurement, timestamp,
                                                newStdDevs);
  } else {
    // frc::DataLogManager::Log(
    //     "WARNING: Vision pose was outside of field! Not adding to
    //     estimator!");
  }
}

void SwerveDrive::UpdateSimulation() {
  std::array<frc::SwerveModuleState, 4> simState;
  int i = 0;
  for (auto& swerveModule : modules) {
    simState[i] = swerveModule.UpdateSimulatedModule(
        frc::RobotController::GetBatteryVoltage());
    i++;
  }

  simStatesPub.Set(simState);

  auto chassisSpeeds =
      consts::swerve::physical::KINEMATICS.ToChassisSpeeds(simState);

  simChassiSpeeds.Set(chassisSpeeds);

  units::radians_per_second_t omega = chassisSpeeds.omega;
  units::radian_t angleChange = omega * (1 / 50_Hz);

  lastSimAngle = lastSimAngle + frc::Rotation2d{angleChange};
  imuSimState.SetRawYaw(lastSimAngle.Degrees());
  imuSimState.SetAngularVelocityZ(omega);
}

void SwerveDrive::UpdateNTEntries() {
  currentStatesPub.Set(moduleStates);
  currentPositionsPub.Set(modulePositions);
  odomUpdateRatePub.Set(odomUpdateRate.value());
  estimatorPub.Set(poseEstimator.GetEstimatedPosition());
  singleTagEstimatorPub.Set(singleTagPoseEstimator.GetEstimatedPosition());
  odomPosePub.Set(odom.GetPose());
  swerveField.SetRobotPose(poseEstimator.GetEstimatedPosition());
  swerveField.GetObject("FL Pos")->SetPose(
      poseEstimator.GetEstimatedPosition().TransformBy(
          frc::Transform2d{consts::swerve::physical::WHEELBASE_LENGTH / 2,
                           consts::swerve::physical::WHEELBASE_WIDTH / 2,
                           frc::Rotation2d{modules[0].GetState().angle}}));
  swerveField.GetObject("FR Pos")->SetPose(
      poseEstimator.GetEstimatedPosition().TransformBy(
          frc::Transform2d{consts::swerve::physical::WHEELBASE_LENGTH / 2,
                           -consts::swerve::physical::WHEELBASE_WIDTH / 2,
                           frc::Rotation2d{modules[1].GetState().angle}}));
  swerveField.GetObject("BL Pos")->SetPose(
      poseEstimator.GetEstimatedPosition().TransformBy(
          frc::Transform2d{-consts::swerve::physical::WHEELBASE_LENGTH / 2,
                           consts::swerve::physical::WHEELBASE_WIDTH / 2,
                           frc::Rotation2d{modules[2].GetState().angle}}));
  swerveField.GetObject("BR Pos")->SetPose(
      poseEstimator.GetEstimatedPosition().TransformBy(
          frc::Transform2d{-consts::swerve::physical::WHEELBASE_LENGTH / 2,
                           -consts::swerve::physical::WHEELBASE_WIDTH / 2,
                           frc::Rotation2d{modules[3].GetState().angle}}));
}

void SwerveDrive::SetupSignals() {
  // If you are changing this you're prob cooked ngl
  for (size_t i = 0; i < modules.size(); i++) {
    const auto& modSigs = modules[i].GetSignals();
    allSignals[(i * 8) + 0] = modSigs[0];
    allSignals[(i * 8) + 1] = modSigs[1];
    allSignals[(i * 8) + 2] = modSigs[2];
    allSignals[(i * 8) + 3] = modSigs[3];
    allSignals[(i * 8) + 4] = modSigs[4];
    allSignals[(i * 8) + 5] = modSigs[5];
    allSignals[(i * 8) + 6] = modSigs[6];
    allSignals[(i * 8) + 7] = modSigs[7];
  }

  allSignals[allSignals.size() - 2] = &imu.GetYaw();
  allSignals[allSignals.size() - 1] = &imu.GetAngularVelocityZWorld();

  ctre::phoenix::StatusCode setFreqStatus =
      ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
          consts::swerve::ODOM_UPDATE_RATE, allSignals);

  frc::DataLogManager::Log(
      fmt::format("Set update frequency for swerve signals. Result was: {}",
                  setFreqStatus.GetName()));

  if (!setFreqStatus.IsOK()) {
    updateFreqAlert.Set(true);
  }

  for (auto& mod : modules) {
    mod.OptimizeBusSignals();
  }

  ctre::phoenix::StatusCode optimizeImuResult = imu.OptimizeBusUtilization();

  frc::DataLogManager::Log(
      fmt::format("Optimized bus signals for imu. Result was: {}",
                  optimizeImuResult.GetName()));

  if (!optimizeImuResult.IsOK()) {
    imuOptimizeAlert.Set(true);
  }
}

void SwerveDrive::ConfigureImu() {
  ctre::phoenix6::configs::Pigeon2Configuration imuConfig;
  imuConfig.MountPose.MountPoseRoll = consts::swerve::physical::IMU_MOUNT_ROLL;
  imuConfig.MountPose.MountPosePitch =
      consts::swerve::physical::IMU_MOUNT_PITCH;
  imuConfig.MountPose.MountPoseYaw = consts::swerve::physical::IMU_MOUNT_YAW;

  ctre::phoenix::StatusCode imuConfigStatus =
      imu.GetConfigurator().Apply(imuConfig);

  frc::DataLogManager::Log(
      fmt::format("Imu Configured. Result was: {}", imuConfigStatus.GetName()));

  if (!imuConfigStatus.IsOK()) {
    imuConfigAlert.Set(true);
  }
}

units::radian_t SwerveDrive::GetYawFromImu() const {
  return yawLatencyComped;
}

void SwerveDrive::ZeroYaw() {
  units::radian_t targetAngle = 0_rad;
  if (str::IsOnRed()) {
    targetAngle = 180_deg;
  } else {
    targetAngle = 0_deg;
  }
  imuZeroAlert.Set(!imu.SetYaw(targetAngle).IsOK());
}

void SwerveDrive::ResetPose(const frc::Pose2d& resetPose) {
  odom.ResetPosition(frc::Rotation2d{GetYawFromImu()}, modulePositions,
                     resetPose);
  poseEstimator.ResetPosition(frc::Rotation2d{GetYawFromImu()}, modulePositions,
                              resetPose);
  singleTagPoseEstimator.ResetPosition(frc::Rotation2d{GetYawFromImu()},
                                       modulePositions, resetPose);
}

void SwerveDrive::DriveFieldRelative(units::meters_per_second_t xVel,
                                     units::meters_per_second_t yVel,
                                     units::radians_per_second_t omega,
                                     bool openLoop) {
  frc::ChassisSpeeds speedsToSend{};

  frc::Rotation2d rot = poseEstimator.GetEstimatedPosition().Rotation();

  if (frc::DriverStation::IsTeleop()) {
    rot = odom.GetPose().Rotation();
  }

  speedsToSend =
      frc::ChassisSpeeds::FromFieldRelativeSpeeds(xVel, yVel, omega, rot);

  Drive(speedsToSend.vx, speedsToSend.vy, speedsToSend.omega, openLoop);
}

void SwerveDrive::Drive(frc::ChassisSpeeds speeds, bool openLoop) {
  Drive(speeds.vx, speeds.vy, speeds.omega, openLoop);
}

void SwerveDrive::Drive(units::meters_per_second_t xVel,
                        units::meters_per_second_t yVel,
                        units::radians_per_second_t omega, bool openLoop) {
  frc::ChassisSpeeds speedsToSend;
  speedsToSend.vx = xVel;
  speedsToSend.vy = yVel;
  speedsToSend.omega = omega;

  // https://github.com/wpilibsuite/allwpilib/issues/7332
  wpi::array<frc::SwerveModuleState, 4> tempStates =
      consts::swerve::physical::KINEMATICS.ToSwerveModuleStates(speedsToSend);
  frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(
      &tempStates, consts::swerve::physical::PHY_CHAR.MaxLinearSpeed());
  frc::ChassisSpeeds speeds =
      consts::swerve::physical::KINEMATICS.ToChassisSpeeds(tempStates);

  speeds = frc::ChassisSpeeds::Discretize(speeds, (1 / 50_Hz));

  std::array<frc::SwerveModuleState, 4> states =
      consts::swerve::physical::KINEMATICS.ToSwerveModuleStates(speeds);

  SetModuleStates(
      states, true, openLoop,
      ConvertModuleForcesToTorqueCurrent(xModuleForce, yModuleForce));
}

void SwerveDrive::SetModuleStates(
    const std::array<frc::SwerveModuleState, 4>& desiredStates, bool optimize,
    bool openLoop, const std::array<units::ampere_t, 4> moduleTorqueCurrentFF) {
  wpi::array<frc::SwerveModuleState, 4> finalState = desiredStates;
  frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(
      &finalState, consts::swerve::physical::PHY_CHAR.MaxLinearSpeed());
  int i = 0;
  for (auto& mod : modules) {
    finalState[i] = mod.GoToState(finalState[i], optimize, openLoop,
                                  moduleTorqueCurrentFF[i]);
    i++;
  }
  desiredStatesPub.Set(finalState);
}

// If loading from pathplanner, the module forces are already in robot relative
// coords
std::array<units::ampere_t, 4> SwerveDrive::ConvertModuleForcesToTorqueCurrent(
    const std::vector<units::newton_t>& xForce,
    const std::vector<units::newton_t>& yForce) {
  std::array<frc::SwerveModuleState, 4> forces;

  std::array<units::ampere_t, 4> retVal;
  for (size_t i = 0; i < xForce.size(); i++) {
    if (xForce[i] == 0_N && yForce[0] == 0_N) {
      break;
    }
    // frc::Translation2d moduleForceFieldRef{units::meter_t{xForce[i].value()},
    //                                        units::meter_t{yForce[i].value()}};
    // frc::Translation2d moduleForceRobotRef =
    //     moduleForceFieldRef.RotateBy(GetPose().Rotation());
    frc::Translation2d moduleForceRobotRef{units::meter_t{xForce[i].value()},
                                           units::meter_t{yForce[i].value()}};
    units::newton_meter_t totalTorqueAtMotor =
        (units::newton_t{moduleForceRobotRef.Norm().value()} *
         consts::swerve::physical::PHY_CHAR.wheelRadius) /
        consts::swerve::physical::PHY_CHAR.driveGearing;
    units::ampere_t expectedTorqueCurrent =
        totalTorqueAtMotor / consts::swerve::physical::PHY_CHAR.driveMotor.Kt;
    retVal[i] = expectedTorqueCurrent;
    forces[i].angle = moduleForceRobotRef.Angle();
    forces[i].speed = units::feet_per_second_t{expectedTorqueCurrent.value()};
  }
  forcesPub.Set(forces);

  return retVal;
}

str::gains::radial::VoltRadialGainsHolder SwerveDrive::GetSteerGains() const {
  return modules[0].GetSteerGains();
}

void SwerveDrive::SetSteerGains(
    str::gains::radial::VoltRadialGainsHolder newGains) {
  for (int i = 0; i < 4; i++) {
    modules[i].SetSteerGains(newGains);
  }
}

DriveGains SwerveDrive::GetDriveGains() const {
  return modules[0].GetDriveGains();
}

void SwerveDrive::SetDriveGains(DriveGains newGains) {
  for (int i = 0; i < 4; i++) {
    modules[i].SetDriveGains(newGains);
  }
}

void SwerveDrive::SetCharacterizationVoltsSteer(units::volt_t volts) {
  modules[0].SetSteerToVoltage(volts);
}

void SwerveDrive::SetCharacterizationAmpsSteer(units::ampere_t amps) {
  modules[0].SetSteerToAmps(amps);
}

void SwerveDrive::SetCharacterizationAmpsDrive(units::ampere_t amps) {
  modules[0].SetDriveToAmps(amps);
  modules[1].SetDriveToAmps(amps);
  modules[2].SetDriveToAmps(amps);
  modules[3].SetDriveToAmps(amps);
}

void SwerveDrive::LogSteerVolts(frc::sysid::SysIdRoutineLog* log) {
  log->Motor("swerve-steer")
      .voltage(units::volt_t{allSignals[7]->GetValueAsDouble()})
      .position(units::turn_t{allSignals[2]->GetValueAsDouble()})
      .velocity(units::turns_per_second_t{allSignals[3]->GetValueAsDouble()});
}

void SwerveDrive::LogSteerTorqueCurrent(frc::sysid::SysIdRoutineLog* log) {
  log->Motor("swerve-steer")
      .voltage(units::volt_t{allSignals[5]->GetValueAsDouble()})
      .position(units::turn_t{allSignals[2]->GetValueAsDouble()})
      .velocity(units::turns_per_second_t{allSignals[3]->GetValueAsDouble()});
}

void SwerveDrive::LogDriveTorqueCurrent(frc::sysid::SysIdRoutineLog* log) {
  log->Motor("swerve-drive")
      .voltage(units::volt_t{allSignals[4]->GetValueAsDouble()})
      .position(units::turn_t{allSignals[0]->GetValueAsDouble()})
      .velocity(units::turns_per_second_t{allSignals[1]->GetValueAsDouble()});
}

std::array<units::radian_t, 4>
SwerveDrive::GetModuleDriveOutputShaftPositions() {
  return {modules[0].GetOutputShaftTurns(), modules[1].GetOutputShaftTurns(),
          modules[2].GetOutputShaftTurns(), modules[3].GetOutputShaftTurns()};
}
