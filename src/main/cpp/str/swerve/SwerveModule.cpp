// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/swerve/SwerveModule.h"

#include <utility>

#include "ctre/phoenix/StatusCodes.h"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/core/CoreCANcoder.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "frc/Alert.h"
#include "frc/DataLogManager.h"
#include "frc/RobotBase.h"
#include "str/swerve/SwerveModuleSim.h"
#include "units/angle.h"
#include "units/current.h"

using namespace str::swerve;

SwerveModule::SwerveModule(const ModuleConstants& consts,
                           const ModulePhysicalCharacteristics& physical,
                           SteerGains steer, DriveGains drive)
    : moduleNamePrefix{consts.moduleName},
      encoderAlertMsg{moduleNamePrefix + " Steer Encoder Configuration"},
      steerAlertMsg{moduleNamePrefix + " Steer Motor Configuration"},
      driveAlertMsg{moduleNamePrefix + " Drive Motor Configuration"},
      optimizeSteerMsg{moduleNamePrefix + " Optimize Steer Signals"},
      optimizeDriveMsg{moduleNamePrefix + " Optimize Drive Signals"},
      configureEncoderAlert(encoderAlertMsg, frc::Alert::AlertType::kError),
      configureSteerAlert(steerAlertMsg, frc::Alert::AlertType::kError),
      configureDriveAlert(driveAlertMsg, frc::Alert::AlertType::kError),
      optimizeSteerMotorAlert(optimizeSteerMsg, frc::Alert::AlertType::kError),
      optimizeDriveMotorAlert(optimizeDriveMsg, frc::Alert::AlertType::kError),
      physicalChar{physical},
      steerGains{std::move(steer)},
      driveGains{std::move(drive)},
      steerEncoder{consts.encoderId, "*"},
      steerMotor{consts.steerId, "*"},
      driveMotor{consts.driveId, "*"},
      moduleSim(consts, physical, driveMotor.GetSimState(),
                steerMotor.GetSimState(), steerEncoder.GetSimState()) {
  ConfigureSteerEncoder(consts.steerEncoderOffset);
  ConfigureSteerMotor(consts.invertSteer, physical.steerGearing,
                      physical.steerSupplySideLimit,
                      physical.steerStatorCurrentLimit);
  ConfigureDriveMotor(consts.invertDrive, physical.driveSupplySideLimit,
                      physical.driveStatorCurrentLimit);
  ConfigureControlSignals();
}

void SwerveModule::OptimizeBusSignals() {
  ctre::phoenix::StatusCode optimizeDriveResult =
      driveMotor.OptimizeBusUtilization();
  frc::DataLogManager::Log(
      fmt::format("Optimized bus signals for {} drive motor. Result was: {}",
                  moduleNamePrefix, optimizeDriveResult.GetName()));
  ctre::phoenix::StatusCode optimizeSteerResult =
      steerMotor.OptimizeBusUtilization();
  frc::DataLogManager::Log(
      fmt::format("Optimized bus signals for {} steer motor. Result was {}",
                  moduleNamePrefix, optimizeSteerResult.GetName()));
  optimizeDriveMotorAlert.Set(!optimizeDriveResult.IsOK());
  optimizeSteerMotorAlert.Set(!optimizeSteerResult.IsOK());
}

frc::SwerveModuleState SwerveModule::GoToState(frc::SwerveModuleState desired,
                                               bool optimize, bool openLoop,
                                               units::ampere_t arbFF) {
  frc::SwerveModuleState currentState = GetState();
  if (optimize) {
    desired.Optimize(currentState.angle);
  }

  desired.CosineScale(currentState.angle);

  steerMotor.SetControl(steerAngleSetter.WithPosition(desired.angle.Radians()));

  units::radians_per_second_t motorSpeed =
      ConvertWheelVelToMotorVel(ConvertLinearVelToWheelVel(desired.speed));

  // Reverse the modules expected backout because of coupling
  units::radians_per_second_t driveBackout =
      steerVelocitySig.GetValue() * physicalChar.couplingRatio;
  motorSpeed += driveBackout;

  if (openLoop) {
    driveMotor.SetControl(
        driveVoltageSetter
            .WithOutput((motorSpeed /
                         ConvertWheelVelToMotorVel(ConvertLinearVelToWheelVel(
                             physicalChar.MaxLinearSpeed()))) *
                        12_V)
            .WithEnableFOC(true));
  } else {
    driveMotor.SetControl(
        driveVelocitySetter.WithVelocity(motorSpeed)
            .WithFeedForward(units::math::copysign(arbFF, motorSpeed)));
  }

  // Just for logging
  desired.speed =
      ConvertWheelVelToLinearVel(ConvertDriveMotorVelToWheelVel(motorSpeed));

  return desired;
}

std::array<ctre::phoenix6::BaseStatusSignal*, 8> SwerveModule::GetSignals() {
  return {&drivePositionSig, &driveVelocitySig,      &steerPositionSig,
          &steerVelocitySig, &driveTorqueCurrentSig, &steerTorqueCurrentSig,
          &driveVoltageSig,  &steerVoltageSig};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  units::radian_t latencyCompSteerPos =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          steerPositionSig, steerVelocitySig);
  units::radian_t latencyCompDrivePos =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          drivePositionSig, driveVelocitySig);

  latencyCompDrivePos -= latencyCompSteerPos * physicalChar.couplingRatio;

  frc::SwerveModulePosition position{
      ConvertWheelRotationsToWheelDistance(
          ConvertDriveMotorRotationsToWheelRotations(latencyCompDrivePos)),
      frc::Rotation2d{latencyCompSteerPos}};

  return position;
}

frc::SwerveModuleState SwerveModule::GetState() {
  frc::SwerveModuleState currentState{
      ConvertWheelVelToLinearVel(
          ConvertDriveMotorVelToWheelVel(driveVelocitySig.GetValue())),
      frc::Rotation2d{steerPositionSig.GetValue()}};

  return currentState;
}

units::radian_t SwerveModule::GetOutputShaftTurns() {
  ctre::phoenix::StatusCode moduleSignalStatus =
      ctre::phoenix6::BaseStatusSignal::WaitForAll(0_s, drivePositionSig,
                                                   driveVelocitySig);

  if (!moduleSignalStatus.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "Error refreshing {} module signal in GetDriveMotorTurns()! "
        "Error was: {}\n",
        moduleNamePrefix, moduleSignalStatus.GetName()));
  }

  units::radian_t latencyCompDrivePos =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          drivePositionSig, driveVelocitySig);

  return ConvertDriveMotorRotationsToWheelRotations(latencyCompDrivePos);
}

frc::SwerveModuleState SwerveModule::UpdateSimulatedModule(
    units::volt_t batteryVoltage) {
  return moduleSim.Update(batteryVoltage);
}

void SwerveModule::SetSteerGains(str::swerve::SteerGains newGains) {
  steerGains = newGains;
  ctre::phoenix6::configs::Slot0Configs steerSlotConfig{};
  steerSlotConfig.kV = steerGains.kV.value();
  steerSlotConfig.kA = steerGains.kA.value();
  steerSlotConfig.kS = steerGains.kS.value();
  steerSlotConfig.kP = steerGains.kP.value();
  steerSlotConfig.kI = steerGains.kI.value();
  steerSlotConfig.kD = steerGains.kD.value();

  ctre::phoenix6::configs::MotionMagicConfigs steerMMConfig{};

  steerMMConfig.MotionMagicCruiseVelocity = steerGains.motionMagicCruiseVel;
  steerMMConfig.MotionMagicExpo_kV = steerGains.motionMagicExpoKv;
  steerMMConfig.MotionMagicExpo_kA = steerGains.motionMagicExpoKa;

  ctre::phoenix::StatusCode statusGains =
      steerMotor.GetConfigurator().Apply(steerSlotConfig);
  if (!statusGains.IsOK()) {
    frc::DataLogManager::Log(
        fmt::format("Swerve Steer Motor was unable to set new gains! "
                    "Error: {}, More Info: {}",
                    statusGains.GetName(), statusGains.GetDescription()));
  }

  ctre::phoenix::StatusCode statusMM =
      steerMotor.GetConfigurator().Apply(steerMMConfig);
  if (!statusMM.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "{} Swerve Steer Motor was unable to set new motion magic config! "
        "Error: {}, More Info: {}",
        moduleNamePrefix, statusMM.GetName(), statusMM.GetDescription()));
  }
}

void SwerveModule::SetDriveGains(str::swerve::DriveGains newGains) {
  driveGains = newGains;
  ctre::phoenix6::configs::Slot0Configs driveSlotConfig{};
  driveSlotConfig.kV = driveGains.kV.value();
  driveSlotConfig.kA = driveGains.kA.value();
  driveSlotConfig.kS = driveGains.kS.value();
  driveSlotConfig.kP = driveGains.kP.value();
  driveSlotConfig.kI = driveGains.kI.value();
  driveSlotConfig.kD = driveGains.kD.value();
  ctre::phoenix::StatusCode status =
      driveMotor.GetConfigurator().Apply(driveSlotConfig);
  if (!status.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "{} Swerve Drive Motor was unable to set new gains! "
        "Error: {}, More Info: {}",
        moduleNamePrefix, status.GetName(), status.GetDescription()));
  }
}

str::swerve::SteerGains SwerveModule::GetSteerGains() const {
  return steerGains;
}

str::swerve::DriveGains SwerveModule::GetDriveGains() const {
  return driveGains;
}

units::ampere_t SwerveModule::GetSimulatedCurrentDraw() const {
  return moduleSim.GetDriveCurrentDraw() + moduleSim.GetSteerCurrentDraw();
}

void SwerveModule::ConfigureSteerEncoder(units::turn_t encoderOffset) {
  ctre::phoenix6::configs::CANcoderConfiguration encoderConfig{};

  encoderConfig.MagnetSensor.MagnetOffset = encoderOffset;
  encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5_tr;
  encoderConfig.MagnetSensor.SensorDirection =
      ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;

  if (frc::RobotBase::IsSimulation()) {
    encoderConfig.MagnetSensor.MagnetOffset = 0_tr;
  }

  ctre::phoenix::StatusCode configResult =
      steerEncoder.GetConfigurator().Apply(encoderConfig);

  frc::DataLogManager::Log(
      fmt::format("Configured steer encoder on {} module. Result was: {}",
                  moduleNamePrefix, configResult.GetName()));

  configureEncoderAlert.Set(!configResult.IsOK());
}

void SwerveModule::ConfigureSteerMotor(bool invert, units::scalar_t gearing,
                                       units::ampere_t supplyLim,
                                       units::ampere_t statorLim) {
  ctre::phoenix6::configs::TalonFXConfiguration steerConfig{};
  ctre::phoenix6::configs::Slot0Configs steerSlotConfig{};

  steerSlotConfig.kA = steerGains.kA.value();
  steerSlotConfig.kV = steerGains.kV.value();
  steerSlotConfig.kS = steerGains.kS.value();
  steerSlotConfig.kP = steerGains.kP.value();
  steerSlotConfig.kI = steerGains.kI.value();
  steerSlotConfig.kD = steerGains.kD.value();
  steerConfig.Slot0 = steerSlotConfig;

  steerConfig.MotionMagic.MotionMagicCruiseVelocity =
      steerGains.motionMagicCruiseVel;
  steerConfig.MotionMagic.MotionMagicExpo_kV = steerGains.motionMagicExpoKv;
  steerConfig.MotionMagic.MotionMagicExpo_kA = steerGains.motionMagicExpoKa;

  steerConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  steerConfig.Feedback.FeedbackRemoteSensorID = steerEncoder.GetDeviceID();
  steerConfig.Feedback.FeedbackSensorSource =
      ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
  steerConfig.Feedback.RotorToSensorRatio = gearing;
  steerConfig.MotorOutput.Inverted =
      invert
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

  steerConfig.TorqueCurrent.PeakForwardTorqueCurrent = statorLim;
  steerConfig.TorqueCurrent.PeakReverseTorqueCurrent = -statorLim;

  steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  steerConfig.CurrentLimits.SupplyCurrentLimit = supplyLim;

  steerConfig.MotorOutput.ControlTimesyncFreqHz = 250_Hz;

  if (frc::RobotBase::IsSimulation()) {
    steerConfig.MotorOutput.Inverted =
        ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  }

  ctre::phoenix::StatusCode configResult =
      steerMotor.GetConfigurator().Apply(steerConfig);

  frc::DataLogManager::Log(
      fmt::format("Configured steer motor on {} module. Result was: {}",
                  moduleNamePrefix, configResult.GetName()));

  configureSteerAlert.Set(!configResult.IsOK());
}

void SwerveModule::ConfigureDriveMotor(bool invert, units::ampere_t supplyLim,
                                       units::ampere_t statorLim) {
  ctre::phoenix6::configs::TalonFXConfiguration driveConfig{};
  ctre::phoenix6::configs::Slot0Configs driveSlotConfig{};

  driveSlotConfig.kV = driveGains.kV.value();
  driveSlotConfig.kA = driveGains.kA.value();
  driveSlotConfig.kS = driveGains.kS.value();
  driveSlotConfig.kP = driveGains.kP.value();
  driveSlotConfig.kI = driveGains.kI.value();
  driveSlotConfig.kD = driveGains.kD.value();
  driveConfig.Slot0 = driveSlotConfig;

  driveConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  driveConfig.MotorOutput.Inverted =
      invert
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = statorLim;
  driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -statorLim;

  driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  driveConfig.CurrentLimits.StatorCurrentLimit = statorLim;

  driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  driveConfig.CurrentLimits.SupplyCurrentLimit = supplyLim;

  driveConfig.MotorOutput.ControlTimesyncFreqHz = 250_Hz;

  if (frc::RobotBase::IsSimulation()) {
    driveConfig.MotorOutput.Inverted =
        ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  }

  ctre::phoenix::StatusCode configResult =
      driveMotor.GetConfigurator().Apply(driveConfig);

  frc::DataLogManager::Log(
      fmt::format("Configured drive motor on {} module. Result was: {}",
                  moduleNamePrefix, configResult.GetName()));

  configureDriveAlert.Set(!configResult.IsOK());
}

void SwerveModule::SetSteerToVoltage(units::volt_t voltsToSend) {
  steerMotor.SetControl(
      steerVoltageSetter.WithOutput(voltsToSend).WithEnableFOC(true));
}

void SwerveModule::SetSteerToAmps(units::ampere_t ampsToSend) {
  steerMotor.SetControl(steerTorqueCurrentSetter.WithOutput(ampsToSend));
}

void SwerveModule::SetDriveToAmps(units::ampere_t ampsToSend) {
  driveMotor.SetControl(driveTorqueCurrentSetter.WithOutput(ampsToSend));
}

void SwerveModule::ConfigureControlSignals() {
  steerAngleSetter.UpdateFreqHz = 0_Hz;
  driveVelocitySetter.UpdateFreqHz = 0_Hz;
  steerVoltageSetter.UpdateFreqHz = 0_Hz;
  driveVoltageSetter.UpdateFreqHz = 0_Hz;
  steerTorqueCurrentSetter.UpdateFreqHz = 0_Hz;
  driveTorqueCurrentSetter.UpdateFreqHz = 0_Hz;
  steerAngleSetter.UseTimesync = true;
  driveVelocitySetter.UseTimesync = true;
  steerVoltageSetter.UseTimesync = true;
  driveVoltageSetter.UseTimesync = true;
  steerTorqueCurrentSetter.UseTimesync = true;
  driveTorqueCurrentSetter.UseTimesync = true;
  steerTorqueCurrentSetter.OverrideCoastDurNeutral = true;
  driveTorqueCurrentSetter.OverrideCoastDurNeutral = true;
  driveVelocitySetter.OverrideCoastDurNeutral = true;
}

units::radian_t SwerveModule::ConvertDriveMotorRotationsToWheelRotations(
    units::radian_t motorRotations) const {
  return motorRotations / physicalChar.driveGearing;
}

units::radians_per_second_t SwerveModule::ConvertDriveMotorVelToWheelVel(
    units::radians_per_second_t motorVel) const {
  return motorVel / physicalChar.driveGearing;
}

units::meter_t SwerveModule::ConvertWheelRotationsToWheelDistance(
    units::radian_t wheelRotations) const {
  return (wheelRotations / 1_rad) * physicalChar.wheelRadius;
}

units::meters_per_second_t SwerveModule::ConvertWheelVelToLinearVel(
    units::radians_per_second_t wheelVel) const {
  return (wheelVel / 1_rad) * physicalChar.wheelRadius;
}

units::radians_per_second_t SwerveModule::ConvertLinearVelToWheelVel(
    units::meters_per_second_t linVel) const {
  return (linVel / physicalChar.wheelRadius) * 1_rad;
}

units::radians_per_second_t SwerveModule::ConvertWheelVelToMotorVel(
    units::radians_per_second_t wheelVel) const {
  return wheelVel * physicalChar.driveGearing;
}
