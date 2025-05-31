// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/L1.h"

#include <string>

#include "constants/L1Constants.h"
#include "ctre/phoenix/StatusCodes.h"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "frc/DataLogManager.h"
#include "frc/RobotBase.h"
#include "frc/RobotController.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "frc2/command/button/Trigger.h"
#include "str/GainTypes.h"
#include "str/Units.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/voltage.h"

L1::L1(str::SuperstructureDisplay& display) : display{display} {
  ConfigureMotors();
  ConfigureControlSignals();

  OptimizeBusSignals();
}

void L1::Periodic() {
  ctre::phoenix::StatusCode status =
      ctre::phoenix6::BaseStatusSignal::RefreshAll(positionSig, velocitySig,
                                                   voltageSig);

  if (!status.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "Error updating L1 positions! Error was: {}", status.GetName()));
  }

  currentAngle = GetPivotAngle();

  isAtGoalAngle = units::math::abs(goalAngle - currentAngle) <
                  consts::l1::gains::ANGLE_TOLERANCE;

  display.SetClimberAngle(currentAngle);
  UpdateNTEntries();
}

void L1::UpdateNTEntries() {
  currentAnglePub.Set(currentAngle.value());
  angleSetpointPub.Set(goalAngle.value());
  isAtSetpointPub.Set(isAtGoalAngle);
}

void L1::SimulationPeriodic() {
  pivotMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  l1Sim.SetInputVoltage(pivotMotorSim.GetMotorVoltage());

  l1Sim.Update(20_ms);

  units::turn_t encPos = l1Sim.GetAngle();
  units::turns_per_second_t encVel = l1Sim.GetVelocity();

  pivotMotorSim.SetRawRotorPosition(encPos * consts::l1::physical::GEARING);
  pivotMotorSim.SetRotorVelocity(encVel * consts::l1::physical::GEARING);
}

units::turn_t L1::GetPivotAngle() {
  units::turn_t latencyCompPosition =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(positionSig,
                                                                   velocitySig);
  return latencyCompPosition;
}

units::turns_per_second_t L1::GetL1Vel() {
  return velocitySig.GetValue();
}

frc2::Trigger L1::IsAtGoalAngle() {
  return frc2::Trigger{[this] { return isAtGoalAngle; }};
}

frc2::CommandPtr L1::Stow() {
  return GoToAngleCmd([] { return consts::l1::physical::CLIMB_STOW_ANGLE; });
}

frc2::CommandPtr L1::Deploy() {
  return GoToAngleCmd([] { return consts::l1::physical::CLIMB_OUT_ANGLE; });
}

frc2::CommandPtr L1::Climb(std::function<units::volt_t()> volts) {
  return frc2::cmd::Run([this, volts] { SetPivotVoltage(volts()); }, {this});
}

frc2::CommandPtr L1::GoToAngleCmd(std::function<units::turn_t()> newAngle) {
  return frc2::cmd::Run([this, newAngle] { GoToAngle(newAngle()); }, {this})
      .Until([this] { return isAtGoalAngle; });
}

void L1::GoToAngle(units::turn_t newAngle) {
  goalAngle = newAngle;
  pivotMotor.SetControl(
      pivotAngleSetter.WithPosition(newAngle).WithEnableFOC(true));
}

void L1::SetPivotVoltage(units::volt_t volts) {
  pivotMotor.SetControl(
      pivotVoltageSetter.WithEnableFOC(true).WithOutput(volts));
}

frc2::CommandPtr L1::TuneL1PID(std::function<bool()> isDone) {
  std::string tablePrefix = "L1/gains/";
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce(
          [tablePrefix, this] {
            frc::SmartDashboard::PutNumber(tablePrefix + "setpoint", 0);
            frc::SmartDashboard::PutNumber(
                tablePrefix + "mmCruiseVel",
                consts::l1::gains::PIVOT_GAINS.motionMagicCruiseVel.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "mmKA",
                consts::l1::gains::PIVOT_GAINS.motionMagicExpoKa.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "mmKV",
                consts ::l1::gains::PIVOT_GAINS.motionMagicExpoKv.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kA", consts::l1::gains::PIVOT_GAINS.kA.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kV", consts::l1::gains::PIVOT_GAINS.kV.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kS", consts::l1::gains::PIVOT_GAINS.kS.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kP", consts::l1::gains::PIVOT_GAINS.kP.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kI", consts::l1::gains::PIVOT_GAINS.kI.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kD", consts::l1::gains::PIVOT_GAINS.kD.value());
            frc::SmartDashboard::PutNumber(tablePrefix + "kG",
                                           consts::l1::gains::kG.value());
            GoToAngle(0_rad);
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

            units::volt_t newKg = units::volt_t{
                frc::SmartDashboard::GetNumber(tablePrefix + "kG", 0)};

            if (newGains != currentGains ||
                !(units::essentiallyEqual(newKg, currentKg, 1e-6))) {
              SetL1Gains(newGains, newKg);
            }

            GoToAngle(units::degree_t{
                frc::SmartDashboard::GetNumber(tablePrefix + "setpoint", 0)});
          },
          {this})
          .Until(isDone));
}

void L1::OptimizeBusSignals() {
  ctre::phoenix::StatusCode freqSetterStatus =
      ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
          consts::l1::BUS_UPDATE_FREQ, positionSig, velocitySig, voltageSig);

  frc::DataLogManager::Log(
      fmt::format("Set bus signal frequenceies for L1. Result was: {}",
                  freqSetterStatus.GetName()));

  signalFrequencyAlert.Set(!freqSetterStatus.IsOK());

  ctre::phoenix::StatusCode optimizeAlgaeIntakeResult =
      pivotMotor.OptimizeBusUtilization();
  frc::DataLogManager::Log(
      fmt::format("Optimized bus signals for L1 motor. Result was: {}",
                  optimizeAlgaeIntakeResult.GetName()));
  optiL1Alert.Set(!optimizeAlgaeIntakeResult.IsOK());
}

void L1::SetL1Gains(str::gains::radial::VoltRadialGainsHolder newGains,
                    units::volt_t kg) {
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
      ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine;
  slotConfig.kG = kg.value();

  ctre::phoenix6::configs::MotionMagicConfigs mmConfig{};

  mmConfig.MotionMagicCruiseVelocity = currentGains.motionMagicCruiseVel;
  mmConfig.MotionMagicExpo_kV = currentGains.motionMagicExpoKv;
  mmConfig.MotionMagicExpo_kA = currentGains.motionMagicExpoKa;

  ctre::phoenix::StatusCode statusGains =
      pivotMotor.GetConfigurator().Apply(slotConfig);
  if (!statusGains.IsOK()) {
    frc::DataLogManager::Log(
        fmt::format("L1 Pivot Motor was unable to set new gains! "
                    "Error: {}, More Info: {}",
                    statusGains.GetName(), statusGains.GetDescription()));
  }

  ctre::phoenix::StatusCode statusMM =
      pivotMotor.GetConfigurator().Apply(mmConfig);
  if (!statusMM.IsOK()) {
    frc::DataLogManager::Log(
        fmt::format("L1 Pivot Motor was unable to set new motion magic config! "
                    "Error: {}, More Info: {}",
                    statusMM.GetName(), statusMM.GetDescription()));
  }
}

void L1::ConfigureMotors() {
  ctre::phoenix6::configs::TalonFXConfiguration config{};
  ctre::phoenix6::configs::Slot0Configs gains{};

  gains.kA = currentGains.kA.value();
  gains.kV = currentGains.kV.value();
  gains.kS = currentGains.kS.value();
  gains.kP = currentGains.kP.value();
  gains.kI = currentGains.kI.value();
  gains.kD = currentGains.kD.value();
  gains.kG = currentKg.value();
  gains.GravityType = ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine;
  config.Slot0 = gains;

  config.MotionMagic.MotionMagicCruiseVelocity =
      currentGains.motionMagicCruiseVel;
  config.MotionMagic.MotionMagicExpo_kV = currentGains.motionMagicExpoKv;
  config.MotionMagic.MotionMagicExpo_kA = currentGains.motionMagicExpoKa;

  config.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  config.MotorOutput.Inverted =
      consts::l1::physical::INVERT_CLIMBER
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  config.TorqueCurrent.PeakForwardTorqueCurrent =
      consts::l1::current_limits::STATOR_LIMIT;
  config.TorqueCurrent.PeakReverseTorqueCurrent =
      -consts::l1::current_limits::STATOR_LIMIT;

  config.CurrentLimits.StatorCurrentLimit =
      consts::l1::current_limits::STATOR_LIMIT;
  config.CurrentLimits.StatorCurrentLimitEnable = true;

  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.CurrentLimits.SupplyCurrentLimit =
      consts::l1::current_limits::SUPPLY_LIMIT;

  if (frc::RobotBase::IsSimulation()) {
    config.MotorOutput.Inverted =
        ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  }
  config.Feedback.SensorToMechanismRatio = consts::l1::physical::GEARING;

  ctre::phoenix::StatusCode configClimberResult =
      pivotMotor.GetConfigurator().Apply(config);

  frc::DataLogManager::Log(fmt::format("Configured l1 motor. Result was: {}",
                                       configClimberResult.GetName()));

  configureL1Alert.Set(!configClimberResult.IsOK());
}

void L1::ConfigureControlSignals() {
  pivotAngleSetter.UpdateFreqHz = 100_Hz;
  pivotVoltageSetter.UpdateFreqHz = 100_Hz;
}
