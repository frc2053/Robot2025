// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/Elevator.h"

#include "constants/ElevatorConstants.h"
#include "ctre/phoenix/StatusCodes.h"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "frc/DataLogManager.h"
#include "frc/RobotBase.h"
#include "frc/RobotController.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/length.h"
#include "units/voltage.h"

Elevator::Elevator(str::SuperstructureDisplay& display) : display{display} {
  ConfigureMotors();
  ConfigureControlSignals();

  OptimizeBusSignals();
}

void Elevator::Periodic() {
  ctre::phoenix::StatusCode status =
      ctre::phoenix6::BaseStatusSignal::WaitForAll(
          2.0 / consts::elevator::BUS_UPDATE_FREQ, leftPositionSig,
          leftVelocitySig, leftTorqueCurrentSig, leftVoltageSig,
          rightPositionSig, rightVelocitySig, rightTorqueCurrentSig,
          rightVoltageSig);

  if (!status.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "Error updating elevator positions! Error was: {}", status.GetName()));
  }

  currentHeight = GetHeight();

  display.SetElevatorHeight(currentHeight);
  UpdateNTEntries();
}

void Elevator::UpdateNTEntries() {
  currentHeightPub.Set(currentHeight.convert<units::inches>().value());
  heightSetpointPub.Set(goalHeight.convert<units::inches>().value());
}

void Elevator::SimulationPeriodic() {
  leftMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  rightMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

  elevatorSim.SetInputVoltage(
      (leftMotorSim.GetMotorVoltage() + rightMotorSim.GetMotorVoltage()) / 2);

  elevatorSim.Update(20_ms);

  units::turn_t pulleyTurns = ConvertHeightToRadians(elevatorSim.GetPosition());
  units::turns_per_second_t pulleyRadialVel =
      ConvertHeightVelToRadianVel(elevatorSim.GetVelocity());

  units::turn_t motorPos = pulleyTurns * consts::elevator::physical::GEARING;

  units::turns_per_second_t motorVel =
      pulleyRadialVel * consts::elevator::physical::GEARING;

  leftMotorSim.SetRawRotorPosition(motorPos);
  leftMotorSim.SetRotorVelocity(motorVel);

  rightMotorSim.SetRawRotorPosition(motorPos);
  rightMotorSim.SetRotorVelocity(motorVel);
}

units::meter_t Elevator::GetHeight() {
  units::turn_t latencyCompLeft =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          leftPositionSig, leftVelocitySig);
  units::turn_t latencyCompRight =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          rightPositionSig, rightVelocitySig);

  units::meter_t avgHeight =
      ConvertRadiansToHeight((latencyCompLeft + latencyCompRight) / 2) *
      consts::elevator::physical::NUM_OF_STAGES;

  return avgHeight;
};

bool Elevator::IsAtGoalHeight() {
  bool isAtHeight = units::math::abs(goalHeight - currentHeight) <
                    consts::elevator::gains::HEIGHT_TOLERANCE;
  return isAtHeight;
}

frc2::CommandPtr Elevator::GoToHeight(
    std::function<units::meter_t()> newHeight) {
  return frc2::cmd::Run([this, newHeight] { GoToHeight(newHeight()); }, {this})
      .Until([this] { return IsAtGoalHeight(); });
}

void Elevator::GoToHeight(units::meter_t newHeight) {
  goalHeight = newHeight;
  leftMotor.SetControl(elevatorHeightSetter.WithPosition(
      ConvertHeightToRadians(newHeight) /
      consts::elevator::physical::NUM_OF_STAGES));
  rightMotor.SetControl(elevatorHeightSetter.WithPosition(
      ConvertHeightToRadians(newHeight) /
      consts::elevator::physical::NUM_OF_STAGES));
}

void Elevator::SetVoltage(units::volt_t volts) {
  leftMotor.SetControl(
      elevatorVoltageSetter.WithEnableFOC(true).WithOutput(volts));
  rightMotor.SetControl(
      elevatorVoltageSetter.WithEnableFOC(true).WithOutput(volts));
}

void Elevator::SetTorqueCurrent(units::ampere_t amps) {
  leftMotor.SetControl(elevatorTorqueCurrentSetter.WithOutput(amps));
  rightMotor.SetControl(elevatorTorqueCurrentSetter.WithOutput(amps));
}

frc2::CommandPtr Elevator::SysIdElevatorQuasistaticVoltage(
    frc2::sysid::Direction dir) {
  return elevatorSysIdVoltage.Quasistatic(dir).WithName(
      "Elevator Quasistatic Voltage");
}
frc2::CommandPtr Elevator::SysIdElevatorDynamicVoltage(
    frc2::sysid::Direction dir) {
  return elevatorSysIdVoltage.Dynamic(dir).WithName("Elevator Dynamic Voltage");
}

frc2::CommandPtr Elevator::SysIdElevatorQuasistaticTorqueCurrent(
    frc2::sysid::Direction dir) {
  return elevatorSysIdTorqueCurrent.Quasistatic(dir).WithName(
      "Elevator Quasistatic Torque Current");
}
frc2::CommandPtr Elevator::SysIdElevatorDynamicTorqueCurrent(
    frc2::sysid::Direction dir) {
  return elevatorSysIdTorqueCurrent.Dynamic(dir).WithName(
      "Elevator Dynamic Torque Current");
}

void Elevator::LogElevatorVolts(frc::sysid::SysIdRoutineLog* log) {
  log->Motor("elevator")
      .voltage((leftVoltageSig.GetValue() + rightVoltageSig.GetValue()) / 2.0)
      .position((leftPositionSig.GetValue() + rightPositionSig.GetValue()) /
                2.0)
      .velocity((leftVelocitySig.GetValue() + rightVelocitySig.GetValue()) /
                2.0);
}

void Elevator::LogElevatorTorqueCurrent(frc::sysid::SysIdRoutineLog* log) {
  log->Motor("elevator")
      .voltage(units::volt_t{(leftTorqueCurrentSig.GetValueAsDouble() +
                              rightTorqueCurrentSig.GetValueAsDouble()) /
                             2.0})
      .position((leftPositionSig.GetValue() + rightPositionSig.GetValue()) /
                2.0)
      .velocity((leftVelocitySig.GetValue() + rightVelocitySig.GetValue()) /
                2.0);
}

void Elevator::OptimizeBusSignals() {
  ctre::phoenix::StatusCode freqSetterStatus =
      ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
          consts::elevator::BUS_UPDATE_FREQ, leftPositionSig, leftVelocitySig,
          leftTorqueCurrentSig, leftVoltageSig, rightPositionSig,
          rightVelocitySig, rightTorqueCurrentSig, rightVoltageSig);

  frc::DataLogManager::Log(
      fmt::format("Set bus signal frequenceies for elevator. Result was: {}",
                  freqSetterStatus.GetName()));

  signalFrequencyAlert.Set(!freqSetterStatus.IsOK());

  ctre::phoenix::StatusCode optimizeLeftResult =
      leftMotor.OptimizeBusUtilization();
  frc::DataLogManager::Log(fmt::format(
      "Optimized bus signals for left elevator motor. Result was: {}",
      optimizeLeftResult.GetName()));
  ctre::phoenix::StatusCode optimizeRightResult =
      rightMotor.OptimizeBusUtilization();
  frc::DataLogManager::Log(fmt::format(
      "Optimized bus signals for right elevator motor. Result was {}",
      optimizeRightResult.GetName()));
  optiLeftAlert.Set(!optimizeLeftResult.IsOK());
  optiRightAlert.Set(!optimizeRightResult.IsOK());
}

void Elevator::SetElevatorGains(str::gains::radial::RadialGainsHolder newGains,
                                units::ampere_t kg) {
  currentGains = newGains;
  ctre::phoenix6::configs::Slot0Configs slotConfig{};
  slotConfig.kV = currentGains.kV.value();
  slotConfig.kA = currentGains.kA.value();
  slotConfig.kS = currentGains.kS.value();
  slotConfig.kP = currentGains.kP.value();
  slotConfig.kI = currentGains.kI.value();
  slotConfig.kD = currentGains.kD.value();
  slotConfig.kG = kg.value();

  ctre::phoenix6::configs::MotionMagicConfigs mmConfig{};

  mmConfig.MotionMagicCruiseVelocity = currentGains.motionMagicCruiseVel;
  mmConfig.MotionMagicExpo_kV = currentGains.motionMagicExpoKv;
  mmConfig.MotionMagicExpo_kA = currentGains.motionMagicExpoKa;

  ctre::phoenix::StatusCode statusGains =
      leftMotor.GetConfigurator().Apply(slotConfig);
  if (!statusGains.IsOK()) {
    frc::DataLogManager::Log(
        fmt::format("Left Elevator Motor was unable to set new gains! "
                    "Error: {}, More Info: {}",
                    statusGains.GetName(), statusGains.GetDescription()));
  }

  ctre::phoenix::StatusCode statusMM =
      leftMotor.GetConfigurator().Apply(mmConfig);
  if (!statusMM.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "Left Elevator Motor was unable to set new motion magic config! "
        "Error: {}, More Info: {}",
        statusMM.GetName(), statusMM.GetDescription()));
  }
}

void Elevator::ConfigureMotors() {
  ctre::phoenix6::configs::TalonFXConfiguration config{};
  ctre::phoenix6::configs::Slot0Configs gains{};

  gains.kA = currentGains.kA.value();
  gains.kV = currentGains.kV.value();
  gains.kS = currentGains.kS.value();
  gains.kP = currentGains.kP.value();
  gains.kI = currentGains.kI.value();
  gains.kD = currentGains.kD.value();
  gains.kG = currentKg.value();
  config.Slot0 = gains;

  config.MotionMagic.MotionMagicCruiseVelocity =
      currentGains.motionMagicCruiseVel;
  config.MotionMagic.MotionMagicExpo_kV = currentGains.motionMagicExpoKv;
  config.MotionMagic.MotionMagicExpo_kA = currentGains.motionMagicExpoKa;

  config.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  config.Feedback.SensorToMechanismRatio = consts::elevator::physical::GEARING;
  config.MotorOutput.Inverted =
      consts::elevator::physical::INVERT_LEFT
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  config.TorqueCurrent.PeakForwardTorqueCurrent =
      consts::elevator::current_limits::STATOR_LIMIT;
  config.TorqueCurrent.PeakReverseTorqueCurrent =
      -consts::elevator::current_limits::STATOR_LIMIT;

  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.CurrentLimits.SupplyCurrentLimit =
      consts::elevator::current_limits::SUPPLY_LIMIT;

  if (frc::RobotBase::IsSimulation()) {
    config.MotorOutput.Inverted =
        ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  }

  ctre::phoenix::StatusCode configLeftResult =
      leftMotor.GetConfigurator().Apply(config);

  frc::DataLogManager::Log(
      fmt::format("Configured left motor on elevator. Result was: {}",
                  configLeftResult.GetName()));

  configureLeftAlert.Set(!configLeftResult.IsOK());

  config.MotorOutput.Inverted =
      consts::elevator::physical::INVERT_RIGHT
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  if (frc::RobotBase::IsSimulation()) {
    config.MotorOutput.Inverted =
        ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  }

  ctre::phoenix::StatusCode configRightResult =
      rightMotor.GetConfigurator().Apply(config);

  frc::DataLogManager::Log(
      fmt::format("Configured right motor on elevator. Result was: {}",
                  configRightResult.GetName()));

  configureRightAlert.Set(!configRightResult.IsOK());
}

void Elevator::ConfigureControlSignals() {
  elevatorHeightSetter.UpdateFreqHz = 0_Hz;
  elevatorVoltageSetter.UpdateFreqHz = 0_Hz;
  elevatorTorqueCurrentSetter.UpdateFreqHz = 0_Hz;
  elevatorTorqueCurrentSetter.OverrideCoastDurNeutral = true;
  elevatorHeightSetter.OverrideCoastDurNeutral = true;
}

units::meter_t Elevator::ConvertRadiansToHeight(units::radian_t rots) {
  return (rots / 1_rad) * (consts::elevator::physical::PULLEY_DIAM / 2);
}

units::radian_t Elevator::ConvertHeightToRadians(units::meter_t height) {
  return 1_rad * (height / (consts::elevator::physical::PULLEY_DIAM / 2));
}

units::meters_per_second_t Elevator::ConvertRadianVelToHeightVel(
    units::radians_per_second_t radialVel) {
  return (radialVel / 1_rad) * (consts::elevator::physical::PULLEY_DIAM / 2);
}

units::radians_per_second_t Elevator::ConvertHeightVelToRadianVel(
    units::meters_per_second_t vel) {
  return 1_rad * (vel / (consts::elevator::physical::PULLEY_DIAM / 2));
}
