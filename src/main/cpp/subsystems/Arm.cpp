// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/Arm.h"

#include "constants/ArmConstants.h"
#include "ctre/phoenix/StatusCodes.h"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "frc/DataLogManager.h"
#include "frc/RobotBase.h"
#include "frc/RobotController.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/voltage.h"

Arm::Arm(str::SuperstructureDisplay& display) : display{display} {
  ConfigureMotors();
  ConfigureControlSignals();

  OptimizeBusSignals();
}

void Arm::Periodic() {
  ctre::phoenix::StatusCode status =
      ctre::phoenix6::BaseStatusSignal::RefreshAll(
          {&pivotPositionSig, &pivotVelocitySig, &pivotTorqueCurrentSig,
           &pivotVoltageSig});

  if (!status.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "Error updating arm positions! Error was: {}", status.GetName()));
  }

  display.SetArmAngle(GetAngle());
}

void Arm::SimulationPeriodic() {
  pivotMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

  armSim.SetInputVoltage(pivotMotorSim.GetMotorVoltage());

  armSim.Update(20_ms);

  units::radian_t motorPos = armSim.GetAngle() * consts::arm::physical::GEARING;

  units::radians_per_second_t motorVel =
      armSim.GetVelocity() * consts::arm::physical::GEARING;

  pivotMotorSim.SetRawRotorPosition(units::turn_t{motorPos});
  pivotMotorSim.SetRotorVelocity(units::turns_per_second_t{motorVel});
}

units::radian_t Arm::GetAngle() {
  units::radian_t latencyCompPivotAngle =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          pivotPositionSig, pivotVelocitySig);

  return latencyCompPivotAngle;
}

void Arm::GoToAngle() {}

void Arm::SetVoltage(units::volt_t volts) {
  pivotMotor.SetControl(
      pivotVoltageSetter.WithEnableFOC(true).WithOutput(volts));
}

void Arm::OptimizeBusSignals() {
  ctre::phoenix::StatusCode freqSetterStatus =
      ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
          consts::arm::BUS_UPDATE_FREQ, pivotPositionSig, pivotVelocitySig,
          pivotTorqueCurrentSig, pivotVoltageSig);

  frc::DataLogManager::Log(
      fmt::format("Set bus signal frequenceies for arm pivot. Result was: {}",
                  freqSetterStatus.GetName()));

  signalFrequencyAlert.Set(!freqSetterStatus.IsOK());

  ctre::phoenix::StatusCode optimizePivotResult =
      pivotMotor.OptimizeBusUtilization();
  frc::DataLogManager::Log(
      fmt::format("Optimized bus signals for arm pivot motor. Result was: {}",
                  optimizePivotResult.GetName()));
  optiPivotAlert.Set(!optimizePivotResult.IsOK());
}

void Arm::ConfigureMotors() {
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
  config.Feedback.SensorToMechanismRatio = consts::arm::physical::GEARING;
  config.MotorOutput.Inverted =
      consts::arm::physical::INVERT_PIVOT
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  config.TorqueCurrent.PeakForwardTorqueCurrent =
      consts::arm::current_limits::STATOR_LIMIT;
  config.TorqueCurrent.PeakReverseTorqueCurrent =
      -consts::arm::current_limits::STATOR_LIMIT;

  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.CurrentLimits.SupplyCurrentLimit =
      consts::arm::current_limits::SUPPLY_LIMIT;

  if (frc::RobotBase::IsSimulation()) {
    config.MotorOutput.Inverted =
        ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  }

  ctre::phoenix::StatusCode configPivotResult =
      pivotMotor.GetConfigurator().Apply(config);

  frc::DataLogManager::Log(fmt::format("Configured arm pivot. Result was: {}",
                                       configPivotResult.GetName()));

  configurePivotAlert.Set(!configPivotResult.IsOK());
}

void Arm::ConfigureControlSignals() {
  pivotAngleSetter.UpdateFreqHz = 0_Hz;
  pivotVoltageSetter.UpdateFreqHz = 0_Hz;
  pivotTorqueCurrentSetter.UpdateFreqHz = 0_Hz;
  pivotTorqueCurrentSetter.OverrideCoastDurNeutral = true;
  pivotAngleSetter.OverrideCoastDurNeutral = true;
}
