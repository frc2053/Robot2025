// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/Pivot.h"

#include <numbers>
#include <string>

#include "constants/PivotConstants.h"
#include "ctre/phoenix/StatusCodes.h"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "frc/DataLogManager.h"
#include "frc/RobotBase.h"
#include "frc/RobotController.h"
#include "frc/controller/ArmFeedforward.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/trajectory/ExponentialProfile.h"
#include "frc/trajectory/TrapezoidProfile.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "frc2/command/button/Trigger.h"
#include "frc2/command/sysid/SysIdRoutine.h"
#include "str/GainTypes.h"
#include "str/Units.h"
#include "units/angle.h"
#include "units/angular_acceleration.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/voltage.h"

Pivot::Pivot(str::SuperstructureDisplay& display) : display{display} {
  ConfigureMotors();
  ConfigureControlSignals();

  OptimizeBusSignals();
}

void Pivot::Periodic() {
  ctre::phoenix::StatusCode status =
      ctre::phoenix6::BaseStatusSignal::RefreshAll(positionSig, velocitySig,
                                                   voltageSig);

  if (!status.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "Error updating pivot positions! Error was: {}", status.GetName()));
  }

  currentAngle = GetAngle();

  auto next = trapProf.Calculate(20_ms, trapSetpoint, trapGoal);

  ffToSend = ff.Calculate(GetAngle().convert<units::radians>(),
                          GetPivotVel().convert<units::radians_per_second>(),
                          next.velocity.convert<units::radians_per_second>());
  frc::SmartDashboard::PutNumber("Kg Applied", ffToSend.value());
  pidOutput = pivotPid.Calculate(GetAngle().value(), next.position.value());

  if (!isCharacterizing) {
    pivotMotor.SetControl(
        pivotVoltageSetter.WithOutput(ffToSend + units::volt_t{pidOutput})
            .WithEnableFOC(true));
  }

  isAtGoalAngle = units::math::abs(trapGoal.position - currentAngle) <
                  consts::pivot::gains::ANGLE_TOLERANCE;

  display.SetPivotAngle(currentAngle);
  UpdateNTEntries();

  trapSetpoint = next;
}

bool Pivot::HasHitFwdSoftLimit() {
  return positionSig.GetValue() >= consts::pivot::physical::SOFT_LIMIT_FWD;
}

bool Pivot::HasHitRevSoftLimit() {
  return positionSig.GetValue() <= consts::pivot::physical::SOFT_LIMIT_REV;
}

void Pivot::SetToStartingPosition() {
  pivotSim.SetInputVoltage(0_V);
  encoderSim.SetRawPosition(83.3_deg);
  encoderSim.SetVelocity(0_deg_per_s);

  pivotMotorSim.SetRawRotorPosition(83.3_deg *
                                    consts::pivot::physical::GEARING);

  pivotSim.SetState(83.3_deg, 0_deg_per_s);
  GoToAngle(83.3_deg, false);
}

void Pivot::UpdateNTEntries() {
  currentAnglePub.Set(currentAngle.convert<units::degrees>().value());
  currentAngleRawPub.Set(
      positionSig.GetValue().convert<units::degrees>().value());
  currentVelPub.Set(GetPivotVel().convert<units::degrees_per_second>().value());
  angularPosSetpointPub.Set(
      trapSetpoint.position.convert<units::degrees>().value());
  angularVelSetpointPub.Set(
      trapSetpoint.velocity.convert<units::degrees_per_second>().value());
  angularGoalPub.Set(trapGoal.position.convert<units::degrees>().value());
  isAtSetpointPub.Set(isAtGoalAngle);
  appliedVoltagePub.Set(voltageSig.GetValue().value());
}

void Pivot::SimulationPeriodic() {
  pivotMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  encoderSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

  pivotSim.SetInputVoltage(pivotMotorSim.GetMotorVoltage());

  pivotSim.Update(20_ms);

  units::turn_t encPos = pivotSim.GetAngle();
  units::turns_per_second_t encVel = pivotSim.GetVelocity();

  encoderSim.SetRawPosition(encPos);
  encoderSim.SetVelocity(encVel);

  pivotMotorSim.SetRawRotorPosition(encPos * consts::pivot::physical::GEARING);
  pivotMotorSim.SetRotorVelocity(encVel * consts::pivot::physical::GEARING);
}

units::turn_t Pivot::GetAngle() {
  units::turn_t latencyCompPosition =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(positionSig,
                                                                   velocitySig);
  return latencyCompPosition;
}

units::turns_per_second_t Pivot::GetPivotVel() {
  return velocitySig.GetValue();
}

frc2::Trigger Pivot::IsClearOfFunnel() {
  return frc2::Trigger{[this] {
    return currentAngle <= consts::pivot::physical::CLEAR_OF_FUNNEL_ANGLE;
  }};
}

frc2::Trigger Pivot::IsAtGoalAngle() {
  return frc2::Trigger{[this] { return isAtGoalAngle; }};
}

frc2::CommandPtr Pivot::Coast() {
  return frc2::cmd::Sequence(
             frc2::cmd::Run([this] { pivotMotor.SetControl(coastSetter); },
                            {this}))
      .IgnoringDisable(true);
}

frc2::CommandPtr Pivot::GoToAngleCmd(std::function<units::turn_t()> newAngle) {
  return GoToAngleCmd(newAngle, [] { return false; });
}

frc2::CommandPtr Pivot::GoToAngleCmd(
    std::function<units::turn_t()> newAngle,
    std::function<bool()> isIntermediateState) {
  return frc2::cmd::Run(
             [this, newAngle, isIntermediateState] {
               GoToAngle(newAngle(), isIntermediateState());
             },
             {this})
      .Until([this] { return isAtGoalAngle; });
}

void Pivot::GoToAngle(units::turn_t newAngle, bool isIntermediateState) {
  if (!units::essentiallyEqual(trapGoal.position, newAngle, 1e-6)) {
    isAtGoalAngle = false;
    units::turns_per_second_t velEnd = 0_rad_per_s;
    if (isIntermediateState) {
      velEnd = consts::pivot::gains::INTERMEDIATE_STATE_MAX_VEL;
    }
    trapGoal = {newAngle, velEnd};
  }
}

void Pivot::SetVoltage(units::volt_t volts) {
  pivotMotor.SetControl(
      pivotVoltageSetter.WithEnableFOC(true).WithOutput(volts));
}

frc2::CommandPtr Pivot::SysIdPivotQuasistaticVoltage(
    frc2::sysid::Direction dir) {
  return pivotSysIdVoltage.Quasistatic(dir)
      .WithName("Pivot Quasistatic Voltage")
      .BeforeStarting([this, dir] {
        std::string dirStr = "Rev";
        if (dir == frc2::sysid::Direction::kForward) {
          dirStr = "Fwd";
        }
        fmt::print("Pivot Sysid Quasi {}\n", dirStr);
        isCharacterizing = true;
      })
      .AndThen([this] { isCharacterizing = false; });
  ;
}
frc2::CommandPtr Pivot::SysIdPivotDynamicVoltage(frc2::sysid::Direction dir) {
  return pivotSysIdVoltage.Dynamic(dir)
      .WithName("Pivot Dynamic Voltage")
      .BeforeStarting([this, dir] {
        std::string dirStr = "Rev";
        if (dir == frc2::sysid::Direction::kForward) {
          dirStr = "Fwd";
        }
        fmt::print("Pivot Sysid Dynamic {}\n", dirStr);
        isCharacterizing = true;
      })
      .AndThen([this] { isCharacterizing = false; });
}

frc2::CommandPtr Pivot::TunePivotPID(std::function<bool()> isDone) {
  std::string tablePrefix = "Pivot/gains/";
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce(
          [tablePrefix, this] {
            frc::SmartDashboard::PutNumber(tablePrefix + "setpoint", 0);
            frc::SmartDashboard::PutNumber(
                tablePrefix + "maxVel",
                maxProfVel.convert<units::degrees_per_second>().value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "maxAccel",
                maxProfAccel.convert<units::degrees_per_second_squared>()
                    .value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kA",
                consts::pivot::gains::PIVOT_GAINS.kA.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kV",
                consts::pivot::gains::PIVOT_GAINS.kV.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kS",
                consts::pivot::gains::PIVOT_GAINS.kS.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kP",
                consts::pivot::gains::PIVOT_GAINS.kP.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kI",
                consts::pivot::gains::PIVOT_GAINS.kI.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kD",
                consts::pivot::gains::PIVOT_GAINS.kD.value());
            frc::SmartDashboard::PutNumber(tablePrefix + "kG",
                                           consts::pivot::gains::kG.value());
            GoToAngle(0_rad, false);
          },
          {this}),
      frc2::cmd::Run(
          [this, tablePrefix] {
            str::gains::radial::VoltRadialGainsHolder newGains{
                units::turns_per_second_t{0},
                str::gains::radial::turn_volt_ka_unit_t{0},
                str::gains::radial::turn_volt_kv_unit_t{0},
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

            units::degrees_per_second_t newMaxVel = units::degrees_per_second_t{
                frc::SmartDashboard::GetNumber(tablePrefix + "maxVel", 0)};
            units::degrees_per_second_squared_t newMaxAccel =
                units::degrees_per_second_squared_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "maxAccel",
                                                   0)};

            units::volt_t newKg = units::volt_t{
                frc::SmartDashboard::GetNumber(tablePrefix + "kG", 0)};

            if (newGains != currentGains ||
                !(units::essentiallyEqual(newKg, currentKg, 1e-6)) ||
                !(units::essentiallyEqual(newMaxVel, maxProfVel, 1e-6)) ||
                !(units::essentiallyEqual(newMaxAccel, maxProfAccel, 1e-6))) {
              SetPivotGains(newGains, newKg, newMaxVel, newMaxAccel);
            }

            GoToAngle(units::degree_t{frc::SmartDashboard::GetNumber(
                          tablePrefix + "setpoint", 0)},
                      false);
          },
          {this})
          .Until(isDone));
}

void Pivot::LogPivotVolts(frc::sysid::SysIdRoutineLog* log) {
  log->Motor("pivot")
      .voltage(voltageSig.GetValue())
      .position(positionSig.GetValue())
      .velocity(velocitySig.GetValue());
}

void Pivot::OptimizeBusSignals() {
  ctre::phoenix::StatusCode freqSetterStatus =
      ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
          consts::pivot::BUS_UPDATE_FREQ, positionSig, velocitySig, voltageSig);

  frc::DataLogManager::Log(
      fmt::format("Set bus signal frequenceies for pivot. Result was: {}",
                  freqSetterStatus.GetName()));

  signalFrequencyAlert.Set(!freqSetterStatus.IsOK());

  ctre::phoenix::StatusCode optimizePivotResult =
      pivotMotor.OptimizeBusUtilization();
  frc::DataLogManager::Log(
      fmt::format("Optimized bus signals for pivot motor. Result was: {}",
                  optimizePivotResult.GetName()));
  optiPivotAlert.Set(!optimizePivotResult.IsOK());
}

void Pivot::SetPivotGains(str::gains::radial::VoltRadialGainsHolder newGains,
                          units::volt_t kg, units::turns_per_second_t newMaxVel,
                          units::turns_per_second_squared_t newMaxAccel) {
  currentGains = newGains;
  currentKg = kg;
  maxProfVel = newMaxVel;
  maxProfAccel = newMaxAccel;

  trapProf = frc::TrapezoidProfile<units::turns>{
      frc::TrapezoidProfile<units::turns>::Constraints{newMaxVel, newMaxAccel}};
  ff = frc::ArmFeedforward{newGains.kS, kg, newGains.kV, newGains.kA};
  pivotPid.SetPID(newGains.kP.value(), newGains.kI.value(),
                  newGains.kD.value());
}

void Pivot::ConfigureMotors() {
  ctre::phoenix6::configs::CANcoderConfiguration encoderCfg{};

  encoderCfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.891113_tr;

  if (frc::RobotBase::IsSimulation()) {
    encoderCfg.MagnetSensor.SensorDirection = ctre::phoenix6::signals::
        SensorDirectionValue::CounterClockwise_Positive;
    encoderCfg.MagnetSensor.MagnetOffset = 0_tr;
  } else {
    encoderCfg.MagnetSensor.SensorDirection =
        ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive;
    encoderCfg.MagnetSensor.MagnetOffset =
        consts::pivot::physical::ENCODER_OFFSET;
  }

  ctre::phoenix::StatusCode configEncoderResult =
      pivotEncoder.GetConfigurator().Apply(encoderCfg);

  frc::DataLogManager::Log(
      fmt::format("Configured pivot encoder. Result was: {}",
                  configEncoderResult.GetName()));

  configureEncoderAlert.Set(!configEncoderResult.IsOK());

  ctre::phoenix6::configs::TalonFXConfiguration config{};

  config.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  config.MotorOutput.Inverted =
      consts::pivot::physical::INVERT_PIVOT
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  config.TorqueCurrent.PeakForwardTorqueCurrent =
      consts::pivot::current_limits::STATOR_LIMIT;
  config.TorqueCurrent.PeakReverseTorqueCurrent =
      -consts::pivot::current_limits::STATOR_LIMIT;

  config.CurrentLimits.StatorCurrentLimitEnable = true;
  config.CurrentLimits.StatorCurrentLimit =
      consts::pivot::current_limits::STATOR_LIMIT;

  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.CurrentLimits.SupplyCurrentLimit =
      consts::pivot::current_limits::SUPPLY_LIMIT;

  if (frc::RobotBase::IsSimulation()) {
    config.MotorOutput.Inverted =
        ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  }

  config.Feedback.FeedbackRemoteSensorID = pivotEncoder.GetDeviceID();
  config.Feedback.FeedbackSensorSource =
      ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
  config.Feedback.SensorToMechanismRatio = 1.0;
  config.Feedback.RotorToSensorRatio = consts::pivot::physical::GEARING;

  config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
  config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
  config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
      consts::pivot::physical::SOFT_LIMIT_FWD;
  config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
      consts::pivot::physical::SOFT_LIMIT_REV;

  ctre::phoenix::StatusCode configPivotResult =
      pivotMotor.GetConfigurator().Apply(config);

  frc::DataLogManager::Log(fmt::format("Configured pivot motor. Result was: {}",
                                       configPivotResult.GetName()));

  configurePivotAlert.Set(!configPivotResult.IsOK());
}

void Pivot::ConfigureControlSignals() {
  pivotVoltageSetter.UpdateFreqHz = 100_Hz;
  coastSetter.UpdateFreqHz = 100_Hz;
}
