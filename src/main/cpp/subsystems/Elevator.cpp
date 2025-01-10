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
#include "units/angle.h"
#include "units/angular_velocity.h"
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

  display.SetElevatorHeight(GetHeight());
}

void Elevator::SimulationPeriodic() {
  leftMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  rightMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

  elevatorSim.SetInputVoltage(
      (leftMotorSim.GetMotorVoltage() + rightMotorSim.GetMotorVoltage()) / 2);

  elevatorSim.Update(20_ms);

  double motorPos =
      (elevatorSim.GetPosition().value() /
       (consts::elevator::physical::PULLEY_DIAM.value() * std::numbers::pi)) *
      (consts::elevator::physical::GEARING);

  double motorVel =
      (elevatorSim.GetVelocity().value() /
       (consts::elevator::physical::PULLEY_DIAM.value() * std::numbers::pi)) *
      (consts::elevator::physical::GEARING);

  leftMotorSim.SetRawRotorPosition(units::turn_t{motorPos});
  leftMotorSim.SetRotorVelocity(units::turns_per_second_t{motorVel});

  rightMotorSim.SetRawRotorPosition(units::turn_t{motorPos});
  rightMotorSim.SetRotorVelocity(units::turns_per_second_t{motorVel});
}

units::meter_t Elevator::GetHeight() {
  units::meter_t latencyCompLeftHeight = units::meter_t{
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          leftPositionSig, leftVelocitySig)
          .value()};
  units::meter_t latencyCompRightHeight = units::meter_t{
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          rightPositionSig, rightVelocitySig)
          .value()};

  units::meter_t avgHeight =
      (latencyCompLeftHeight + latencyCompRightHeight) / 2;

  return avgHeight;
};

void Elevator::GoToHeight() {}

void Elevator::SetVoltage(units::volt_t volts) {
  leftMotor.SetControl(
      elevatorVoltageSetter.WithEnableFOC(true).WithOutput(volts));
  rightMotor.SetControl(
      elevatorVoltageSetter.WithEnableFOC(true).WithOutput(volts));
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

void Elevator::ConfigureMotors() {
  ctre::phoenix6::configs::TalonFXConfiguration config{};
  ctre::phoenix6::configs::Slot0Configs gains{};

  gains.kA = currentGains.kA.value();
  gains.kV = currentGains.kV.value();
  gains.kS = currentGains.kS.value();
  gains.kP = currentGains.kP.value();
  gains.kI = currentGains.kI.value();
  gains.kD = currentGains.kD.value();
  config.Slot0 = gains;

  // Maybe scuffed because I want the talon to report linear distance
  config.MotionMagic.MotionMagicCruiseVelocity =
      units::turns_per_second_t{currentGains.motionMagicCruiseVel.value()};
  config.MotionMagic.MotionMagicExpo_kV =
      ctre::unit::volts_per_turn_per_second_t{
          currentGains.motionMagicExpoKv.value()};
  config.MotionMagic.MotionMagicExpo_kA =
      ctre::unit::volts_per_turn_per_second_squared_t{
          currentGains.motionMagicExpoKa.value()};

  config.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  config.Feedback.SensorToMechanismRatio =
      consts::elevator::physical::GEARING /
      (std::numbers::pi * consts::elevator::physical::PULLEY_DIAM.value());
  config.MotorOutput.Inverted =
      consts::elevator::physical::INVERT_LEFT
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  // config.TorqueCurrent.PeakForwardTorqueCurrent =
  //     consts::elevator::current_limits::STATOR_LIMIT;
  // config.TorqueCurrent.PeakReverseTorqueCurrent =
  //     -consts::elevator::current_limits::STATOR_LIMIT;

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
