// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/Climber.h"

#include <string>

#include "constants/ClimberConstants.h"
#include "ctre/phoenix/StatusCodes.h"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "frc/DataLogManager.h"
#include "frc/RobotBase.h"
#include "frc/RobotController.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "frc2/command/button/Trigger.h"
#include "str/GainTypes.h"
#include "str/Units.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/voltage.h"

Climber::Climber(str::SuperstructureDisplay& display) : display{display} {
  ConfigureMotors();
  ConfigureControlSignals();

  OptimizeBusSignals();
}

void Climber::Periodic() {
  ctre::phoenix::StatusCode status =
      ctre::phoenix6::BaseStatusSignal::RefreshAll(positionSig, velocitySig,
                                                   voltageSig);

  if (!status.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "Error updating Climber positions! Error was: {}", status.GetName()));
  }

  currentAngle = GetClimberAngle();

  isAtGoalAngle = units::math::abs(goalAngle - currentAngle) <
                  consts::climber::gains::ANGLE_TOLERANCE;

  display.SetClimberAngle(currentAngle);
  UpdateNTEntries();
}

void Climber::UpdateNTEntries() {
  currentAnglePub.Set(currentAngle.convert<units::degrees>().value());
  angleSetpointPub.Set(goalAngle.convert<units::degrees>().value());
  isAtSetpointPub.Set(isAtGoalAngle);
}

void Climber::SimulationPeriodic() {
  climberMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  climberSim.SetInputVoltage(climberMotorSim.GetMotorVoltage());

  climberSim.Update(20_ms);

  units::turn_t encPos = climberSim.GetAngle();
  units::turns_per_second_t encVel = climberSim.GetVelocity();

  climberMotorSim.SetRawRotorPosition(encPos *
                                      consts::climber::physical::GEARING);
  climberMotorSim.SetRotorVelocity(encVel * consts::climber::physical::GEARING);
}

units::turn_t Climber::GetClimberAngle() {
  units::turn_t latencyCompPosition =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(positionSig,
                                                                   velocitySig);
  return latencyCompPosition;
}

units::turns_per_second_t Climber::GetClimberVel() {
  return velocitySig.GetValue();
}

frc2::Trigger Climber::IsAtGoalAngle() {
  return frc2::Trigger{[this] { return isAtGoalAngle; }};
}

frc2::CommandPtr Climber::Stow() {
  return GoToAngleCmd(
      [] { return consts::climber::physical::CLIMB_STOW_ANGLE; });
}

frc2::CommandPtr Climber::Deploy() {
  return GoToAngleCmd(
      [] { return consts::climber::physical::CLIMB_OUT_ANGLE; });
}

frc2::CommandPtr Climber::Climb(std::function<units::volt_t()> volts) {
  return frc2::cmd::Run([this, volts] { SetClimberVoltage(volts()); }, {this});
}

frc2::CommandPtr Climber::GoToAngleCmd(
    std::function<units::turn_t()> newAngle) {
  return frc2::cmd::Run([this, newAngle] { GoToAngle(newAngle()); }, {this})
      .Until([this] { return isAtGoalAngle; });
}

void Climber::GoToAngle(units::turn_t newAngle) {
  goalAngle = newAngle;
  climberMotor.SetControl(
      climberAngleSetter.WithPosition(newAngle).WithEnableFOC(true));
}

void Climber::SetClimberVoltage(units::volt_t volts) {
  climberMotor.SetControl(
      climberVoltageSetter.WithEnableFOC(true).WithOutput(volts));
}

frc2::CommandPtr Climber::TuneClimberPID(std::function<bool()> isDone) {
  std::string tablePrefix = "Climber/gains/";
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce(
          [tablePrefix, this] {
            frc::SmartDashboard::PutNumber(tablePrefix + "setpoint", 0);
            frc::SmartDashboard::PutNumber(tablePrefix + "mmCruiseVel",
                                           consts::climber::gains::CLIMBER_GAINS
                                               .motionMagicCruiseVel.value());
            frc::SmartDashboard::PutNumber(tablePrefix + "mmKA",
                                           consts::climber::gains::CLIMBER_GAINS
                                               .motionMagicExpoKa.value());
            frc::SmartDashboard::PutNumber(tablePrefix + "mmKV",
                                           consts::climber::gains::CLIMBER_GAINS
                                               .motionMagicExpoKv.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kA",
                consts::climber::gains::CLIMBER_GAINS.kA.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kV",
                consts::climber::gains::CLIMBER_GAINS.kV.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kS",
                consts::climber::gains::CLIMBER_GAINS.kS.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kP",
                consts::climber::gains::CLIMBER_GAINS.kP.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kI",
                consts::climber::gains::CLIMBER_GAINS.kI.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kD",
                consts::climber::gains::CLIMBER_GAINS.kD.value());
            frc::SmartDashboard::PutNumber(tablePrefix + "kG",
                                           consts::climber::gains::kG.value());
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
              SetClimberGains(newGains, newKg);
            }

            GoToAngle(units::degree_t{
                frc::SmartDashboard::GetNumber(tablePrefix + "setpoint", 0)});
          },
          {this})
          .Until(isDone));
}

void Climber::OptimizeBusSignals() {
  ctre::phoenix::StatusCode freqSetterStatus =
      ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
          consts::climber::BUS_UPDATE_FREQ, positionSig, velocitySig,
          voltageSig);

  frc::DataLogManager::Log(
      fmt::format("Set bus signal frequenceies for climber. Result was: {}",
                  freqSetterStatus.GetName()));

  signalFrequencyAlert.Set(!freqSetterStatus.IsOK());

  ctre::phoenix::StatusCode optimizeAlgaeIntakeResult =
      climberMotor.OptimizeBusUtilization();
  frc::DataLogManager::Log(
      fmt::format("Optimized bus signals for climber motor. Result was: {}",
                  optimizeAlgaeIntakeResult.GetName()));
  optiClimberAlert.Set(!optimizeAlgaeIntakeResult.IsOK());
}

void Climber::SetClimberGains(
    str::gains::radial::VoltRadialGainsHolder newGains, units::volt_t kg) {
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
      climberMotor.GetConfigurator().Apply(slotConfig);
  if (!statusGains.IsOK()) {
    frc::DataLogManager::Log(
        fmt::format("Climber Motor was unable to set new gains! "
                    "Error: {}, More Info: {}",
                    statusGains.GetName(), statusGains.GetDescription()));
  }

  ctre::phoenix::StatusCode statusMM =
      climberMotor.GetConfigurator().Apply(mmConfig);
  if (!statusMM.IsOK()) {
    frc::DataLogManager::Log(
        fmt::format("Climber Motor was unable to set new motion magic config! "
                    "Error: {}, More Info: {}",
                    statusMM.GetName(), statusMM.GetDescription()));
  }
}

void Climber::ConfigureMotors() {
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
      consts::climber::physical::INVERT_CLIMBER
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  config.TorqueCurrent.PeakForwardTorqueCurrent =
      consts::climber::current_limits::STATOR_LIMIT;
  config.TorqueCurrent.PeakReverseTorqueCurrent =
      -consts::climber::current_limits::STATOR_LIMIT;

  config.CurrentLimits.StatorCurrentLimit =
      consts::climber::current_limits::STATOR_LIMIT;
  config.CurrentLimits.StatorCurrentLimitEnable = true;

  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.CurrentLimits.SupplyCurrentLimit =
      consts::climber::current_limits::SUPPLY_LIMIT;

  if (frc::RobotBase::IsSimulation()) {
    config.MotorOutput.Inverted =
        ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  }
  config.Feedback.SensorToMechanismRatio = consts::climber::physical::GEARING;

  ctre::phoenix::StatusCode configClimberResult =
      climberMotor.GetConfigurator().Apply(config);

  frc::DataLogManager::Log(
      fmt::format("Configured climber motor. Result was: {}",
                  configClimberResult.GetName()));

  configureClimberAlert.Set(!configClimberResult.IsOK());
}

void Climber::ConfigureControlSignals() {
  climberAngleSetter.UpdateFreqHz = 100_Hz;
  climberVoltageSetter.UpdateFreqHz = 100_Hz;
}
