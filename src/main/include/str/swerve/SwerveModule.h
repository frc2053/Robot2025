// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/Alert.h>
#include <units/angle.h>

#include <string>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include "SwerveModuleHelpers.h"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/controls/TorqueCurrentFOC.hpp"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "str/swerve/SwerveModuleHelpers.h"
#include "str/swerve/SwerveModuleSim.h"
#include "units/current.h"
#include "units/dimensionless.h"
#include "units/velocity.h"
#include "units/voltage.h"

namespace str::swerve {

class SwerveModule {
 public:
  explicit SwerveModule(const ModuleConstants& consts,
                        const ModulePhysicalCharacteristics& physical,
                        str::gains::radial::AmpRadialGainsHolder steer,
                        DriveGains drive);
  void OptimizeBusSignals();
  frc::SwerveModuleState GoToState(frc::SwerveModuleState desired,
                                   bool optimize, bool openLoop,
                                   units::ampere_t arbFF);
  std::array<ctre::phoenix6::BaseStatusSignal*, 8> GetSignals();
  frc::SwerveModulePosition GetPosition();
  frc::SwerveModuleState GetState();
  units::radian_t GetOutputShaftTurns();
  frc::SwerveModuleState UpdateSimulatedModule(units::volt_t batteryVoltage);
  void SetSteerGains(str::gains::radial::AmpRadialGainsHolder newGains);
  void SetDriveGains(str::swerve::DriveGains newGains);
  str::gains::radial::AmpRadialGainsHolder GetSteerGains() const;
  str::swerve::DriveGains GetDriveGains() const;
  units::ampere_t GetSimulatedCurrentDraw() const;
  void SetSteerToAmps(units::ampere_t ampsToSend);
  void SetSteerToVoltage(units::volt_t voltsToSend);
  void SetDriveToAmps(units::ampere_t ampsToSend);

 private:
  void ConfigureSteerEncoder(units::turn_t encoderOffset);
  void ConfigureSteerMotor(bool invert, units::scalar_t gearing,
                           units::ampere_t supplyLim,
                           units::ampere_t statorLim);
  void ConfigureDriveMotor(bool invert, units::ampere_t supplyLim,
                           units::ampere_t statorLim);
  void ConfigureControlSignals();
  units::radian_t ConvertDriveMotorRotationsToWheelRotations(
      units::radian_t motorRotations) const;
  units::radians_per_second_t ConvertDriveMotorVelToWheelVel(
      units::radians_per_second_t motorVel) const;
  units::meter_t ConvertWheelRotationsToWheelDistance(
      units::radian_t wheelRotations) const;
  units::meters_per_second_t ConvertWheelVelToLinearVel(
      units::radians_per_second_t wheelVel) const;
  units::radians_per_second_t ConvertLinearVelToWheelVel(
      units::meters_per_second_t linVel) const;
  units::radians_per_second_t ConvertWheelVelToMotorVel(
      units::radians_per_second_t wheelVel) const;

  std::string moduleNamePrefix;

  std::string encoderAlertMsg;
  std::string steerAlertMsg;
  std::string driveAlertMsg;
  std::string optimizeSteerMsg;
  std::string optimizeDriveMsg;
  frc::Alert configureEncoderAlert;
  frc::Alert configureSteerAlert;
  frc::Alert configureDriveAlert;
  frc::Alert optimizeSteerMotorAlert;
  frc::Alert optimizeDriveMotorAlert;

  ModulePhysicalCharacteristics physicalChar;
  str::gains::radial::AmpRadialGainsHolder steerGains;
  DriveGains driveGains;

  ctre::phoenix6::hardware::CANcoder steerEncoder;
  ctre::phoenix6::hardware::TalonFX steerMotor;
  ctre::phoenix6::hardware::TalonFX driveMotor;

  ctre::phoenix6::StatusSignal<units::turn_t> steerPositionSig =
      steerMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> steerVelocitySig =
      steerMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::ampere_t> steerTorqueCurrentSig =
      steerMotor.GetTorqueCurrent();
  ctre::phoenix6::StatusSignal<units::volt_t> steerVoltageSig =
      steerMotor.GetMotorVoltage();
  ctre::phoenix6::StatusSignal<units::turn_t> drivePositionSig =
      driveMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> driveVelocitySig =
      driveMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::ampere_t> driveTorqueCurrentSig =
      driveMotor.GetTorqueCurrent();
  ctre::phoenix6::StatusSignal<units::volt_t> driveVoltageSig =
      driveMotor.GetMotorVoltage();

  ctre::phoenix6::controls::MotionMagicExpoTorqueCurrentFOC steerAngleSetter{
      0_rad};
  ctre::phoenix6::controls::VelocityTorqueCurrentFOC driveVelocitySetter{
      0_rad_per_s};

  ctre::phoenix6::controls::VoltageOut steerVoltageSetter{0_V};
  ctre::phoenix6::controls::VoltageOut driveVoltageSetter{0_V};
  ctre::phoenix6::controls::TorqueCurrentFOC steerTorqueCurrentSetter{0_A};
  ctre::phoenix6::controls::TorqueCurrentFOC driveTorqueCurrentSetter{0_A};

  SwerveModuleSim moduleSim;
};
}  // namespace str::swerve
