// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/Elevator.h"

#include <string>

#include "constants/ElevatorConstants.h"
#include "ctre/phoenix/StatusCodes.h"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "frc/DataLogManager.h"
#include "frc/RobotBase.h"
#include "frc/RobotController.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "str/Units.h"
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

  // sim is not inverted
  if (frc::RobotBase::IsSimulation()) {
    rightMotor.SetControl(followerSetter.WithOpposeMasterDirection(false));
  } else {
    rightMotor.SetControl(followerSetter);
  }

  currentHeight = GetHeight();

  isAtGoalHeight = units::math::abs(goalHeight - currentHeight) <
                   consts::elevator::gains::HEIGHT_TOLERANCE;

  display.SetElevatorHeight(currentHeight);
  UpdateNTEntries();
}

void Elevator::UpdateNTEntries() {
  currentHeightPub.Set(currentHeight.convert<units::inches>().value());
  heightSetpointPub.Set(goalHeight.convert<units::inches>().value());
  isAtSetpointPub.Set(isAtGoalHeight);
}

void Elevator::SimulationPeriodic() {
  leftMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  rightMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  outputEncoderSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

  elevatorSim.SetInputVoltage(
      (leftMotorSim.GetMotorVoltage() + rightMotorSim.GetMotorVoltage()) / 2.0);

  elevatorSim.Update(20_ms);

  units::turn_t encPos = ConvertHeightToEncPos(elevatorSim.GetPosition());

  units::turns_per_second_t encVel =
      ConvertHeightVelToEncVel(elevatorSim.GetVelocity());

  outputEncoderSim.SetRawPosition(encPos);
  outputEncoderSim.SetVelocity(encVel);

  leftMotorSim.SetRawRotorPosition(encPos *
                                   consts::elevator::physical::GEARING);
  leftMotorSim.SetRotorVelocity(encVel * consts::elevator::physical::GEARING);

  rightMotorSim.SetRawRotorPosition(encPos *
                                    consts::elevator::physical::GEARING);
  rightMotorSim.SetRotorVelocity(encVel * consts::elevator::physical::GEARING);
}

units::meter_t Elevator::GetHeight() {
  units::turn_t latencyCompLeft =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          leftPositionSig, leftVelocitySig);
  units::turn_t latencyCompRight =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          rightPositionSig, rightVelocitySig);

  units::meter_t avgHeight =
      ConvertEncPosToHeight((latencyCompLeft + latencyCompRight) / 2.0);

  return avgHeight;
}

units::meters_per_second_t Elevator::GetElevatorVel() {
  units::meters_per_second_t avgVel = ConvertEncVelToHeightVel(
      (leftVelocitySig.GetValue() + rightVelocitySig.GetValue()) / 2.0);
  return avgVel;
}

bool Elevator::IsAtGoalHeight() {
  return isAtGoalHeight;
}

frc2::CommandPtr Elevator::GoToHeightCmd(
    std::function<units::meter_t()> newHeight) {
  return frc2::cmd::Run([this, newHeight] { GoToHeight(newHeight()); }, {this})
      .Until([this] { return IsAtGoalHeight(); });
}

void Elevator::GoToHeight(units::meter_t newHeight) {
  goalHeight = newHeight;
  leftMotor.SetControl(
      elevatorHeightSetter.WithPosition(ConvertHeightToEncPos(newHeight)));
}

void Elevator::SetVoltage(units::volt_t volts) {
  leftMotor.SetControl(
      elevatorVoltageSetter.WithEnableFOC(true).WithOutput(volts));
}

void Elevator::SetTorqueCurrent(units::ampere_t amps) {
  leftMotor.SetControl(elevatorTorqueCurrentSetter.WithOutput(amps));
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

frc2::CommandPtr Elevator::TuneElevatorPID(std::function<bool()> isDone) {
  std::string tablePrefix = "Elevator/gains/";
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce(
          [tablePrefix, this] {
            frc::SmartDashboard::PutNumber(tablePrefix + "setpoint", 0);
            frc::SmartDashboard::PutNumber(
                tablePrefix + "mmCruiseVel",
                consts::elevator::gains::ELEVATOR_GAINS.motionMagicCruiseVel
                    .value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "mmKA", consts::elevator::gains::ELEVATOR_GAINS
                                          .motionMagicExpoKa.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "mmKV", consts::elevator::gains::ELEVATOR_GAINS
                                          .motionMagicExpoKv.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kA",
                consts::elevator::gains::ELEVATOR_GAINS.kA.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kV",
                consts::elevator::gains::ELEVATOR_GAINS.kV.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kS",
                consts::elevator::gains::ELEVATOR_GAINS.kS.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kP",
                consts::elevator::gains::ELEVATOR_GAINS.kP.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kI",
                consts::elevator::gains::ELEVATOR_GAINS.kI.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kD",
                consts::elevator::gains::ELEVATOR_GAINS.kD.value());
            frc::SmartDashboard::PutNumber(tablePrefix + "kG",
                                           consts::elevator::gains::kG.value());
            GoToHeight(0_m);
          },
          {this}),
      frc2::cmd::Run(
          [this, tablePrefix] {
            str::gains::radial::RadialGainsHolder newGains{
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

            units::ampere_t newKg = units::ampere_t{
                frc::SmartDashboard::GetNumber(tablePrefix + "kG", 0)};

            if (newGains != currentGains ||
                !(units::essentiallyEqual(newKg, currentKg, 1e-6))) {
              SetElevatorGains(newGains, newKg);
            }

            GoToHeight(units::inch_t{
                frc::SmartDashboard::GetNumber(tablePrefix + "setpoint", 0)});
          },
          {this})
          .Until(isDone));
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
  currentKg = kg;
  ctre::phoenix6::configs::Slot0Configs slotConfig{};
  slotConfig.kV = currentGains.kV.value();
  slotConfig.kA = currentGains.kA.value();
  slotConfig.kS = currentGains.kS.value();
  slotConfig.kP = currentGains.kP.value();
  slotConfig.kI = currentGains.kI.value();
  slotConfig.kD = currentGains.kD.value();
  slotConfig.GravityType =
      ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
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
  ctre::phoenix6::configs::CANcoderConfiguration encCfg{};

  encCfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1_tr;
  encCfg.MagnetSensor.SensorDirection =
      ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
  if (frc::RobotBase::IsSimulation()) {
    encCfg.MagnetSensor.MagnetOffset = 0_tr;
  } else {
    encCfg.MagnetSensor.MagnetOffset = consts::elevator::physical::ENC_OFFSET;
  }
  outputEncoder.GetConfigurator().Apply(encCfg);

  ctre::phoenix6::configs::TalonFXConfiguration config{};
  ctre::phoenix6::configs::Slot0Configs gains{};

  gains.kA = currentGains.kA.value();
  gains.kV = currentGains.kV.value();
  gains.kS = currentGains.kS.value();
  gains.kP = currentGains.kP.value();
  gains.kI = currentGains.kI.value();
  gains.kD = currentGains.kD.value();
  gains.kG = currentKg.value();
  gains.GravityType =
      ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
  config.Slot0 = gains;

  config.MotionMagic.MotionMagicCruiseVelocity =
      currentGains.motionMagicCruiseVel;
  config.MotionMagic.MotionMagicExpo_kV = currentGains.motionMagicExpoKv;
  config.MotionMagic.MotionMagicExpo_kA = currentGains.motionMagicExpoKa;

  config.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
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

  config.Feedback.FeedbackRemoteSensorID = outputEncoder.GetDeviceID();
  config.Feedback.FeedbackSensorSource =
      ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
  config.Feedback.SensorToMechanismRatio = 1.0;
  config.Feedback.RotorToSensorRatio = consts::elevator::physical::GEARING;

  ctre::phoenix::StatusCode configLeftResult =
      leftMotor.GetConfigurator().Apply(config);

  frc::DataLogManager::Log(
      fmt::format("Configured left motor on elevator. Result was: {}",
                  configLeftResult.GetName()));

  configureLeftAlert.Set(!configLeftResult.IsOK());

  // Empty config because we only want to follow left
  ctre::phoenix6::configs::TalonFXConfiguration rightConfig{};

  rightConfig.Feedback.FeedbackRemoteSensorID = outputEncoder.GetDeviceID();
  rightConfig.Feedback.FeedbackSensorSource =
      ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
  rightConfig.Feedback.SensorToMechanismRatio = 1.0;
  rightConfig.Feedback.RotorToSensorRatio = consts::elevator::physical::GEARING;
  ctre::phoenix::StatusCode configRightResult =
      rightMotor.GetConfigurator().Apply(rightConfig);

  frc::DataLogManager::Log(
      fmt::format("Configured right motor on elevator. Result was: {}",
                  configRightResult.GetName()));

  configureRightAlert.Set(!configRightResult.IsOK());
}

void Elevator::ConfigureControlSignals() {
  elevatorHeightSetter.UpdateFreqHz = 0_Hz;
  elevatorVoltageSetter.UpdateFreqHz = 0_Hz;
  elevatorTorqueCurrentSetter.UpdateFreqHz = 0_Hz;
  followerSetter.UpdateFreqHz = 0_Hz;
  elevatorTorqueCurrentSetter.OverrideCoastDurNeutral = true;
  elevatorHeightSetter.OverrideCoastDurNeutral = true;
}

units::meter_t Elevator::ConvertEncPosToHeight(units::turn_t turns) {
  units::meter_t ret = (turns / 1_rad) *
                       (consts::elevator::physical::PULLEY_DIAM / 2.0) *
                       consts::elevator::physical::NUM_OF_STAGES;
  return ret;
}

units::turn_t Elevator::ConvertHeightToEncPos(units::meter_t height) {
  units::turn_t ret =
      1_rad * (height / (consts::elevator::physical::PULLEY_DIAM / 2.0)) /
      consts::elevator::physical::NUM_OF_STAGES;
  return ret;
}

units::meters_per_second_t Elevator::ConvertEncVelToHeightVel(
    units::turns_per_second_t radialVel) {
  units::meters_per_second_t ret =
      (radialVel / 1_rad) * (consts::elevator::physical::PULLEY_DIAM / 2.0) *
      consts::elevator::physical::NUM_OF_STAGES;
  return ret;
}

units::turns_per_second_t Elevator::ConvertHeightVelToEncVel(
    units::meters_per_second_t vel) {
  units::turns_per_second_t ret =
      1_rad * (vel / (consts::elevator::physical::PULLEY_DIAM / 2.0)) /
      consts::elevator::physical::NUM_OF_STAGES;
  return ret;
}
