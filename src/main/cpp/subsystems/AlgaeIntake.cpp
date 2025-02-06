// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/AlgaeIntake.h"

#include <string>

#include "constants/AlgaeIntakeConstants.h"
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

AlgaeIntake::AlgaeIntake(str::SuperstructureDisplay& display)
    : display{display} {
  ConfigureMotors();
  ConfigureControlSignals();

  OptimizeBusSignals();

  nt->SetDefaultBoolean("SimGrabbingAlgae", false);
}

void AlgaeIntake::Periodic() {
  ctre::phoenix::StatusCode status =
      ctre::phoenix6::BaseStatusSignal::RefreshAll(
          positionSig, velocitySig, voltageSig, positionRollerSig,
          velocityRollerSig, voltageRollerSig, torqueCurrentRollerSig);

  if (!status.IsOK()) {
    frc::DataLogManager::Log(
        fmt::format("Error updating AlgaeIntake positions! Error was: {}",
                    status.GetName()));
  }

  currentAngle = GetAlgaePivotAngle();
  currentTorque = torqueCurrentRollerSig.GetValue();

  isAtGoalAngle = units::math::abs(goalAngle - currentAngle) <
                  consts::algae::gains::ANGLE_TOLERANCE;

  previouslyHadAlgae = hasAlgae;

  if (frc::RobotBase::IsSimulation()) {
    if (fakeAlgaeSuck) {
      currentTorque = 400_A;
    }
  }

  filteredCurrent = currentFilter.Calculate(currentTorque);
  bool intakeSpike = intakeSpikeDebouncer.Calculate(
      filteredCurrent > consts::algae::current_limits::GRABBED_ALGAE);

  hasAlgae = intakeSpike || previouslyHadAlgae;

  display.SetAlgaeIntakeAngle(currentAngle);
  UpdateNTEntries();
}

void AlgaeIntake::UpdateNTEntries() {
  currentAnglePub.Set(currentAngle.convert<units::degrees>().value());
  angleSetpointPub.Set(goalAngle.convert<units::degrees>().value());
  isAtSetpointPub.Set(isAtGoalAngle);
  rollerTorquePub.Set(currentTorque.value());
  filteredCurrentPub.Set(filteredCurrent.value());
  hasAlgaePub.Set(hasAlgae);
}

void AlgaeIntake::SimulationPeriodic() {
  fakeAlgaeSuck = gotAlgaeSub.Get();

  algaeMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  algaePivotSim.SetInputVoltage(algaeMotorSim.GetMotorVoltage());
  rollerSim.SetInputVoltage(algaeRollerMotorSim.GetMotorVoltage());

  algaePivotSim.Update(20_ms);
  rollerSim.Update(20_ms);

  units::turn_t encPos = algaePivotSim.GetAngle();
  units::turns_per_second_t encVel = algaePivotSim.GetVelocity();

  algaeMotorSim.SetRawRotorPosition(encPos *
                                    consts::algae::physical::PIVOT_GEARING);
  algaeMotorSim.SetRotorVelocity(encVel *
                                 consts::algae::physical::PIVOT_GEARING);

  algaeRollerMotorSim.SetRotorVelocity(rollerSim.GetAngularVelocity() *
                                       consts::algae::physical::ROLLER_GEARING);
}

units::radian_t AlgaeIntake::GetAlgaePivotAngle() {
  units::turn_t latencyCompPosition =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(positionSig,
                                                                   velocitySig);
  return latencyCompPosition;
}

units::radians_per_second_t AlgaeIntake::GetAlgaePivotVel() {
  return velocitySig.GetValue();
}

frc2::Trigger AlgaeIntake::IsAtGoalAngle() {
  return frc2::Trigger{[this] { return isAtGoalAngle; }};
}

frc2::CommandPtr AlgaeIntake::Stow() {
  return frc2::cmd::Parallel(
      GoToAngleCmd([] { return consts::algae::physical::ALGAE_STOW_ANGLE; }),
      frc2::cmd::RunOnce([this] { algaeRollerMotor.Set(0); }));
}

frc2::CommandPtr AlgaeIntake::Poop() {
  return frc2::cmd::Parallel(
             GoToAngleCmd([] {
               return consts::algae::physical::ALGAE_INTAKE_ANGLE;
             }).Repeatedly(),
             frc2::cmd::RunOnce([this] {
               hasAlgae = false;
               previouslyHadAlgae = false;
               algaeRollerMotor.Set(-1);
             }))
      .FinallyDo(
          [this] { frc2::CommandScheduler::GetInstance().Schedule(Stow()); });
}

frc2::CommandPtr AlgaeIntake::Hold() {
  return frc2::cmd::Sequence(
      GoToAngleCmd([] { return consts::algae::physical::ALGAE_HOLD_ANGLE; }));
}

frc2::CommandPtr AlgaeIntake::Intake() {
  return Roller().FinallyDo([this] {
    frc2::CommandScheduler::GetInstance().Schedule(
        frc2::cmd::Either(Hold(), Stow(), [this] { return hasAlgae; }));
  });
}

frc2::CommandPtr AlgaeIntake::Roller() {
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce(
          [this] {
            GoToAngle(consts::algae::physical::ALGAE_INTAKE_ANGLE);
            algaeRollerMotor.Set(1.0);
          },
          {this}),
      frc2::cmd::WaitUntil([this] { return hasAlgae; }),
      frc2::cmd::RunOnce([this] { algaeRollerMotor.Set(0); }, {this}));
}

frc2::CommandPtr AlgaeIntake::GoToAngleCmd(
    std::function<units::radian_t()> newAngle) {
  return frc2::cmd::Run([this, newAngle] { GoToAngle(newAngle()); }, {this})
      .Until([this] { return isAtGoalAngle; });
}

void AlgaeIntake::GoToAngle(units::radian_t newAngle) {
  if (!units::essentiallyEqual(goalAngle, newAngle, 1e-6)) {
    isAtGoalAngle = false;
    goalAngle = newAngle;
  }
  algaePivotMotor.SetControl(
      algaePivotAngleSetter.WithPosition(newAngle).WithEnableFOC(true));
}

void AlgaeIntake::SetPivotVoltage(units::volt_t volts) {
  algaePivotMotor.SetControl(
      algaePivotVoltageSetter.WithEnableFOC(true).WithOutput(volts));
}

frc2::CommandPtr AlgaeIntake::SysIdAlgaePivotQuasistaticVoltage(
    frc2::sysid::Direction dir) {
  return algaePivotSysIdVoltage.Quasistatic(dir).WithName(
      "Algae Pivot Intake Quasistatic Voltage");
}
frc2::CommandPtr AlgaeIntake::SysIdAlgaePivotDynamicVoltage(
    frc2::sysid::Direction dir) {
  return algaePivotSysIdVoltage.Dynamic(dir).WithName(
      "Algae Pivot Intake Dynamic Voltage");
}

frc2::CommandPtr AlgaeIntake::TuneAlgaePivotPID(std::function<bool()> isDone) {
  std::string tablePrefix = "AlgaePivot/gains/";
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce(
          [tablePrefix, this] {
            frc::SmartDashboard::PutNumber(tablePrefix + "setpoint", 0);
            frc::SmartDashboard::PutNumber(
                tablePrefix + "mmCruiseVel",
                consts::algae::gains::ALGAE_PIVOT_GAINS.motionMagicCruiseVel
                    .value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "mmKA", consts::algae::gains::ALGAE_PIVOT_GAINS
                                          .motionMagicExpoKa.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "mmKV", consts::algae::gains::ALGAE_PIVOT_GAINS
                                          .motionMagicExpoKv.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kA",
                consts::algae::gains::ALGAE_PIVOT_GAINS.kA.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kV",
                consts::algae::gains::ALGAE_PIVOT_GAINS.kV.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kS",
                consts::algae::gains::ALGAE_PIVOT_GAINS.kS.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kP",
                consts::algae::gains::ALGAE_PIVOT_GAINS.kP.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kI",
                consts::algae::gains::ALGAE_PIVOT_GAINS.kI.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kD",
                consts::algae::gains::ALGAE_PIVOT_GAINS.kD.value());
            frc::SmartDashboard::PutNumber(tablePrefix + "kG",
                                           consts::algae::gains::kG.value());
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
              SetAlgaePivotGains(newGains, newKg);
            }

            GoToAngle(units::degree_t{
                frc::SmartDashboard::GetNumber(tablePrefix + "setpoint", 0)});
          },
          {this})
          .Until(isDone));
}

void AlgaeIntake::LogAlgaePivotVolts(frc::sysid::SysIdRoutineLog* log) {
  log->Motor("algaepivot")
      .voltage(voltageSig.GetValue())
      .position(positionSig.GetValue())
      .velocity(velocitySig.GetValue());
}

void AlgaeIntake::OptimizeBusSignals() {
  ctre::phoenix::StatusCode freqSetterStatus =
      ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
          consts::algae::BUS_UPDATE_FREQ, positionSig, velocitySig, voltageSig,
          positionRollerSig, velocityRollerSig, voltageRollerSig,
          torqueCurrentRollerSig);

  frc::DataLogManager::Log(fmt::format(
      "Set bus signal frequenceies for algae intake. Result was: {}",
      freqSetterStatus.GetName()));

  signalFrequencyAlert.Set(!freqSetterStatus.IsOK());

  ctre::phoenix::StatusCode optimizeAlgaeIntakeResult =
      algaePivotMotor.OptimizeBusUtilization();
  frc::DataLogManager::Log(
      fmt::format("Optimized bus signals for algae pivot motor. Result was: {}",
                  optimizeAlgaeIntakeResult.GetName()));
  optiAlgaeAlert.Set(!optimizeAlgaeIntakeResult.IsOK());
}

void AlgaeIntake::SetAlgaePivotGains(
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
      algaePivotMotor.GetConfigurator().Apply(slotConfig);
  if (!statusGains.IsOK()) {
    frc::DataLogManager::Log(
        fmt::format("Algae Pivot Motor was unable to set new gains! "
                    "Error: {}, More Info: {}",
                    statusGains.GetName(), statusGains.GetDescription()));
  }

  ctre::phoenix::StatusCode statusMM =
      algaePivotMotor.GetConfigurator().Apply(mmConfig);
  if (!statusMM.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "AlgaeIntake Motor was unable to set new motion magic config! "
        "Error: {}, More Info: {}",
        statusMM.GetName(), statusMM.GetDescription()));
  }
}

void AlgaeIntake::ConfigureMotors() {
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
      consts::algae::physical::INVERT_PIVOT
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  config.TorqueCurrent.PeakForwardTorqueCurrent =
      consts::algae::current_limits::STATOR_LIMIT;
  config.TorqueCurrent.PeakReverseTorqueCurrent =
      -consts::algae::current_limits::STATOR_LIMIT;

  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.CurrentLimits.SupplyCurrentLimit =
      consts::algae::current_limits::SUPPLY_LIMIT;

  if (frc::RobotBase::IsSimulation()) {
    config.MotorOutput.Inverted =
        ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  }
  config.Feedback.SensorToMechanismRatio =
      consts::algae::physical::PIVOT_GEARING;

  ctre::phoenix::StatusCode configAlgaePivotResult =
      algaePivotMotor.GetConfigurator().Apply(config);

  frc::DataLogManager::Log(
      fmt::format("Configured algae pivot motor. Result was: {}",
                  configAlgaePivotResult.GetName()));

  configureAlgaePivotAlert.Set(!configAlgaePivotResult.IsOK());

  ctre::phoenix6::configs::TalonFXConfiguration rollerConfig{};

  rollerConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  rollerConfig.MotorOutput.Inverted =
      consts::algae::physical::INVERT_ROLLER
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  rollerConfig.TorqueCurrent.PeakForwardTorqueCurrent =
      consts::algae::current_limits::STATOR_LIMIT;
  rollerConfig.TorqueCurrent.PeakReverseTorqueCurrent =
      -consts::algae::current_limits::STATOR_LIMIT;

  rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  rollerConfig.CurrentLimits.SupplyCurrentLimit =
      consts::algae::current_limits::SUPPLY_LIMIT;

  if (frc::RobotBase::IsSimulation()) {
    rollerConfig.MotorOutput.Inverted =
        ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  }

  ctre::phoenix::StatusCode configAlgaeRollerResult =
      algaeRollerMotor.GetConfigurator().Apply(rollerConfig);

  frc::DataLogManager::Log(
      fmt::format("Configured algae roller motor. Result was: {}",
                  configAlgaeRollerResult.GetName()));

  configureAlgaePivotAlert.Set(!configAlgaePivotResult.IsOK());
  configureAlgaeRollerAlert.Set(!configAlgaeRollerResult.IsOK());
}

void AlgaeIntake::ConfigureControlSignals() {
  algaePivotAngleSetter.UpdateFreqHz = 100_Hz;
  algaePivotVoltageSetter.UpdateFreqHz = 100_Hz;
}
