// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/Elevator.h"

#include <string>

#include "constants/ElevatorConstants.h"
#include "ctre/phoenix/StatusCodes.h"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/configs/Configs.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "frc/DataLogManager.h"
#include "frc/RobotBase.h"
#include "frc/RobotController.h"
#include "frc/controller/ElevatorFeedforward.h"
#include "frc/trajectory/ExponentialProfile.h"
#include "frc/trajectory/TrapezoidProfile.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "frc2/command/button/Trigger.h"
#include "str/GainTypes.h"
#include "str/Units.h"
#include "units/acceleration.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/length.h"
#include "units/time.h"
#include "units/velocity.h"
#include "units/voltage.h"

Elevator::Elevator(str::SuperstructureDisplay& display) : display{display} {
  ConfigureMotors();
  ConfigureControlSignals();

  OptimizeBusSignals();

  frontMotor.SetPosition(0_tr);
  backMotor.SetPosition(0_tr);
}

void Elevator::Periodic() {
  ctre::phoenix::StatusCode status =
      ctre::phoenix6::BaseStatusSignal::WaitForAll(
          2.0 / consts::elevator::BUS_UPDATE_FREQ, frontPositionSig,
          frontVelocitySig, frontVoltageSig, backPositionSig, backVelocitySig,
          backVoltageSig);

  if (!status.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "Error updating elevator positions! Error was: {}", status.GetName()));
  }

  backMotor.SetControl(followerSetter);

  currentHeight = GetHeight();

  auto next = trapProf.Calculate(20_ms, trapSetpoint, trapGoal);

  ffToSend = ff.Calculate(GetElevatorVel(), next.velocity);
  pidOutput = elevatorPid.Calculate(GetHeight().value(), next.position.value());

  if (!isCharacterizing) {
    frontMotor.SetControl(
        elevatorVoltageSetter.WithOutput(ffToSend + units::volt_t{pidOutput})
            .WithEnableFOC(true));
  }

  isAtGoalHeight = units::math::abs(trapGoal.position - currentHeight) <
                   consts::elevator::gains::HEIGHT_TOLERANCE;

  display.SetElevatorHeight(currentHeight);
  UpdateNTEntries();

  trapSetpoint = next;
}

void Elevator::UpdateNTEntries() {
  currentHeightPub.Set(currentHeight.convert<units::inches>().value());
  currentVelPub.Set(
      GetElevatorVel()
          .convert<units::compound_unit<units::inches,
                                        units::inverse<units::seconds>>>()
          .value());
  heightGoalPub.Set(trapGoal.position.convert<units::inches>().value());
  heightPosSetpointPub.Set(
      trapSetpoint.position.convert<units::inches>().value());
  heightVelSetpointPub.Set(
      trapSetpoint.velocity
          .convert<units::compound_unit<units::inches,
                                        units::inverse<units::seconds>>>()
          .value());
  isAtSetpointPub.Set(isAtGoalHeight);
}

void Elevator::SimulationPeriodic() {
  frontMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  backMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

  elevatorSim.SetInputVoltage(
      (frontMotorSim.GetMotorVoltage() + backMotorSim.GetMotorVoltage()) / 2.0);

  elevatorSim.Update(20_ms);

  units::turn_t encPos = ConvertHeightToEncPos(elevatorSim.GetPosition());

  units::turns_per_second_t encVel =
      ConvertHeightVelToEncVel(elevatorSim.GetVelocity());

  frontMotorSim.SetRawRotorPosition(encPos *
                                    consts::elevator::physical::GEARING);
  frontMotorSim.SetRotorVelocity(encVel * consts::elevator::physical::GEARING);

  backMotorSim.SetRawRotorPosition(encPos *
                                   consts::elevator::physical::GEARING);
  backMotorSim.SetRotorVelocity(encVel * consts::elevator::physical::GEARING);
}

units::meter_t Elevator::GetHeight() {
  units::turn_t latencyCompFront =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          frontPositionSig, frontVelocitySig);
  units::turn_t latencyCompBack =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          backPositionSig, backVelocitySig);

  units::meter_t avgHeight =
      ConvertEncPosToHeight((latencyCompFront + latencyCompBack) / 2.0) *
      consts::elevator::physical::NUM_OF_STAGES;

  return avgHeight;
}

units::meters_per_second_t Elevator::GetElevatorVel() {
  units::meters_per_second_t avgVel =
      ConvertEncVelToHeightVel(
          (frontVelocitySig.GetValue() + backVelocitySig.GetValue()) / 2.0) *
      consts::elevator::physical::NUM_OF_STAGES;
  return avgVel;
}

frc2::Trigger Elevator::IsAtGoalHeight() {
  return frc2::Trigger{[this] { return isAtGoalHeight; }};
}

frc2::CommandPtr Elevator::Coast() {
  return frc2::cmd::Sequence(frc2::cmd::Run(
                                 [this] {
                                   frontMotor.SetControl(coastSetter);
                                   backMotor.SetControl(coastSetter);
                                 },
                                 {this}))
      .IgnoringDisable(true);
}

frc2::CommandPtr Elevator::GoToHeightCmd(
    std::function<units::meter_t()> newHeight) {
  return frc2::cmd::Run([this, newHeight] { GoToHeight(newHeight()); }, {this})
      .Until([this] { return isAtGoalHeight; });
}

void Elevator::GoToHeight(units::meter_t newHeight) {
  if (!units::essentiallyEqual(trapGoal.position, newHeight, 1e-6)) {
    isAtGoalHeight = false;
    trapGoal = {newHeight, 0_mps};
  }
}

void Elevator::SetVoltage(units::volt_t volts) {
  frontMotor.SetControl(
      elevatorVoltageSetter.WithEnableFOC(true).WithOutput(volts));
}

frc2::CommandPtr Elevator::SysIdElevatorQuasistaticVoltage(
    frc2::sysid::Direction dir) {
  return elevatorSysIdVoltage.Quasistatic(dir)
      .WithName("Elevator Quasistatic Voltage")
      .BeforeStarting([this] { isCharacterizing = true; })
      .AndThen([this] { isCharacterizing = false; });
}
frc2::CommandPtr Elevator::SysIdElevatorDynamicVoltage(
    frc2::sysid::Direction dir) {
  return elevatorSysIdVoltage.Dynamic(dir)
      .WithName("Elevator Dynamic Voltage")
      .BeforeStarting([this] { isCharacterizing = true; })
      .AndThen([this] { isCharacterizing = false; });
}

frc2::CommandPtr Elevator::TuneElevatorPID(std::function<bool()> isDone) {
  std::string tablePrefix = "Elevator/gains/";
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce(
          [tablePrefix, this] {
            frc::SmartDashboard::PutNumber(tablePrefix + "setpoint", 0);
            frc::SmartDashboard::PutNumber(
                tablePrefix + "maxVel",
                maxProfVel.convert<units::feet_per_second>().value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "maxAccel",
                maxProfAccel.convert<units::feet_per_second_squared>().value());
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
            str::gains::linear::VoltLinearGainsHolder newGains{
                units::meters_per_second_t{0},
                str::gains::linear::meter_volt_ka_unit_t{0},
                str::gains::linear::meter_volt_kv_unit_t{0},
                str::gains::linear::meter_volt_ka_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kA", 0)},
                str::gains::linear::meter_volt_kv_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kV", 0)},
                units::volt_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kS", 0)},
                str::gains::linear::meter_volt_kp_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kP", 0)},
                str::gains::linear::meter_volt_ki_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kI", 0)},
                str::gains::linear::meter_volt_kd_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kD", 0)}};

            units::volt_t newKg = units::volt_t{
                frc::SmartDashboard::GetNumber(tablePrefix + "kG", 0)};

            units::meters_per_second_t newMaxVel = units::feet_per_second_t{
                frc::SmartDashboard::GetNumber(tablePrefix + "maxVel", 0)};
            units::meters_per_second_squared_t newMaxAccel =
                units::feet_per_second_squared_t{frc::SmartDashboard::GetNumber(
                    tablePrefix + "maxAccel", 0)};

            if (newGains != currentGains ||
                !(units::essentiallyEqual(newKg, currentKg, 1e-6)) ||
                !(units::essentiallyEqual(newMaxVel, maxProfVel, 1e-6)) ||
                !(units::essentiallyEqual(newMaxAccel, maxProfAccel, 1e-6))) {
              SetElevatorGains(newGains, newKg, newMaxVel, newMaxAccel);
            }

            GoToHeight(units::inch_t{
                frc::SmartDashboard::GetNumber(tablePrefix + "setpoint", 0)});
          },
          {this})
          .Until(isDone));
}

void Elevator::LogElevatorVolts(frc::sysid::SysIdRoutineLog* log) {
  log->Motor("elevator")
      .voltage((frontVoltageSig.GetValue() + backVoltageSig.GetValue()) / 2.0)
      .position((frontPositionSig.GetValue() + backPositionSig.GetValue()) /
                2.0)
      .velocity((frontVelocitySig.GetValue() + backVelocitySig.GetValue()) /
                2.0);
}

void Elevator::OptimizeBusSignals() {
  ctre::phoenix::StatusCode freqSetterStatus =
      ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
          consts::elevator::BUS_UPDATE_FREQ, frontPositionSig, frontVelocitySig,
          frontVoltageSig, backPositionSig, backVelocitySig, backVoltageSig);

  frc::DataLogManager::Log(
      fmt::format("Set bus signal frequenceies for elevator. Result was: {}",
                  freqSetterStatus.GetName()));

  signalFrequencyAlert.Set(!freqSetterStatus.IsOK());

  ctre::phoenix::StatusCode optimizeFrontResult =
      frontMotor.OptimizeBusUtilization();
  frc::DataLogManager::Log(fmt::format(
      "Optimized bus signals for front elevator motor. Result was: {}",
      optimizeFrontResult.GetName()));
  ctre::phoenix::StatusCode optimizeBackResult =
      backMotor.OptimizeBusUtilization();
  frc::DataLogManager::Log(fmt::format(
      "Optimized bus signals for back elevator motor. Result was {}",
      optimizeBackResult.GetName()));
  optiFrontAlert.Set(!optimizeFrontResult.IsOK());
  optiBackAlert.Set(!optimizeBackResult.IsOK());
}

void Elevator::SetElevatorGains(
    str::gains::linear::VoltLinearGainsHolder newGains, units::volt_t kg,
    units::meters_per_second_t newMaxVel,
    units::meters_per_second_squared_t newMaxAccel) {
  currentGains = newGains;
  currentKg = kg;
  maxProfAccel = newMaxAccel;
  maxProfVel = newMaxVel;

  trapProf = frc::TrapezoidProfile<units::meter>{{newMaxVel, newMaxAccel}};
  ff = frc::ElevatorFeedforward{newGains.kS, kg, newGains.kV, newGains.kA};
  elevatorPid.SetPID(newGains.kP.value(), newGains.kI.value(),
                     newGains.kD.value());
}

void Elevator::ConfigureMotors() {
  ctre::phoenix6::configs::TalonFXConfiguration config{};

  config.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  config.MotorOutput.Inverted =
      consts::elevator::physical::INVERT_FRONT
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  config.TorqueCurrent.PeakForwardTorqueCurrent =
      consts::elevator::current_limits::STATOR_LIMIT;
  config.TorqueCurrent.PeakReverseTorqueCurrent =
      -consts::elevator::current_limits::STATOR_LIMIT;

  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.CurrentLimits.SupplyCurrentLimit =
      consts::elevator::current_limits::SUPPLY_LIMIT;

  config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
  config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0_tr;

  if (frc::RobotBase::IsSimulation()) {
    config.MotorOutput.Inverted =
        ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  }

  config.Feedback.SensorToMechanismRatio = consts::elevator::physical::GEARING;

  ctre::phoenix::StatusCode configFrontResult =
      frontMotor.GetConfigurator().Apply(config);

  frc::DataLogManager::Log(
      fmt::format("Configured front motor on elevator. Result was: {}",
                  configFrontResult.GetName()));

  configureFrontAlert.Set(!configFrontResult.IsOK());

  // Empty config because we only want to follow front
  ctre::phoenix6::configs::TalonFXConfiguration backConfig{};
  backConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;

  backConfig.Feedback.SensorToMechanismRatio =
      consts::elevator::physical::GEARING;

  ctre::phoenix::StatusCode configBackResult =
      backMotor.GetConfigurator().Apply(backConfig);

  frc::DataLogManager::Log(
      fmt::format("Configured back motor on elevator. Result was: {}",
                  configBackResult.GetName()));

  configureBackAlert.Set(!configBackResult.IsOK());
}

void Elevator::SetToZeroHeight() {
  elevatorSim.SetInputVoltage(0_V);
  frontMotorSim.SetRawRotorPosition(0_tr);
  frontMotorSim.SetRotorVelocity(0_deg_per_s);

  backMotorSim.SetRawRotorPosition(0_tr);
  backMotorSim.SetRotorVelocity(0_deg_per_s);

  elevatorSim.SetState(0_m, 0_mps);
  GoToHeight(0_m);
}

void Elevator::ConfigureControlSignals() {
  elevatorVoltageSetter.UpdateFreqHz = 250_Hz;
  followerSetter.UpdateFreqHz = 250_Hz;
  coastSetter.UpdateFreqHz = 250_Hz;
}

units::meter_t Elevator::ConvertEncPosToHeight(units::turn_t turns) {
  units::meter_t ret =
      (turns / 1_rad) * (consts::elevator::physical::PULLEY_DIAM / 2.0);
  return ret;
}

units::turn_t Elevator::ConvertHeightToEncPos(units::meter_t height) {
  units::turn_t ret =
      1_rad * (height / (consts::elevator::physical::PULLEY_DIAM / 2.0));
  return ret;
}

units::meters_per_second_t Elevator::ConvertEncVelToHeightVel(
    units::turns_per_second_t radialVel) {
  units::meters_per_second_t ret =
      (radialVel / 1_rad) * (consts::elevator::physical::PULLEY_DIAM / 2.0);
  return ret;
}

units::turns_per_second_t Elevator::ConvertHeightVelToEncVel(
    units::meters_per_second_t vel) {
  units::turns_per_second_t ret =
      1_rad * (vel / (consts::elevator::physical::PULLEY_DIAM / 2.0));
  return ret;
}
