// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <memory>
#include <string>

#include <ctre/phoenix6/TalonFX.hpp>

#include "constants/AlgaeIntakeConstants.h"
#include "ctre/phoenix6/SignalLogger.hpp"
#include "ctre/phoenix6/controls/MotionMagicExpoVoltage.hpp"
#include "frc/Alert.h"
#include "frc/filter/LinearFilter.h"
#include "frc/simulation/FlywheelSim.h"
#include "frc/simulation/SingleJointedArmSim.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/button/Trigger.h"
#include "frc2/command/sysid/SysIdRoutine.h"
#include "networktables/BooleanTopic.h"
#include "networktables/DoubleTopic.h"
#include "networktables/NetworkTableInstance.h"
#include "str/GainTypes.h"
#include "str/SuperstructureDisplay.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/voltage.h"
#include "frc/system/plant/LinearSystemId.h"

class AlgaeIntake : public frc2::SubsystemBase {
 public:
  explicit AlgaeIntake(str::SuperstructureDisplay& display);  // Is this
                                                              // correct?
  void OptimizeBusSignals();
  void Periodic() override;
  void SimulationPeriodic() override;
  units::radian_t GetAlgaePivotAngle();
  void GoToAngle(units::radian_t newAngle);
  frc2::Trigger IsAtGoalAngle();
  void SetPivotVoltage(units::volt_t volts);
  frc2::CommandPtr Stow();
  frc2::CommandPtr Hold();
  frc2::CommandPtr Intake();
  frc2::CommandPtr Roller();
  frc2::CommandPtr Poop();
  bool HasAlgae() { return hasAlgae; }

  frc2::CommandPtr GoToAngleCmd(std::function<units::radian_t()> newAngle);
  frc2::CommandPtr SysIdAlgaePivotQuasistaticVoltage(
      frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdAlgaePivotDynamicVoltage(frc2::sysid::Direction dir);
  frc2::CommandPtr TuneAlgaePivotPID(std::function<bool()> isDone);

 private:
  void ConfigureMotors();
  void ConfigureControlSignals();
  void UpdateNTEntries();
  units::radians_per_second_t GetAlgaePivotVel();
  void SetAlgaePivotGains(str::gains::radial::VoltRadialGainsHolder newGains,
                          units::volt_t kg);

  void LogAlgaePivotVolts(frc::sysid::SysIdRoutineLog* log);

  ctre::phoenix6::hardware::TalonFX algaePivotMotor{
      consts::algae::can_ids::ALGAE_PIVOT_MOTOR};

  ctre::phoenix6::hardware::TalonFX algaeRollerMotor{
      consts::algae::can_ids::ALGAE_ROLLER_MOTOR};

  units::radian_t goalAngle = 0_rad;
  units::radian_t currentAngle = 0_rad;
  bool isAtGoalAngle = false;
  units::ampere_t currentTorque = 0_A;
  units::ampere_t filteredCurrent{0_A};
  bool hasAlgae = false;
  bool previouslyHadAlgae = false;
  bool fakeAlgaeSuck{false};

  ctre::phoenix6::sim::TalonFXSimState& algaeMotorSim =
      algaePivotMotor.GetSimState();
  // The Roller Motor
  ctre::phoenix6::sim::TalonFXSimState& algaeRollerMotorSim =
      algaeRollerMotor.GetSimState();

  ctre::phoenix6::StatusSignal<units::turn_t> positionSig =
      algaePivotMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> velocitySig =
      algaePivotMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::volt_t> voltageSig =
      algaePivotMotor.GetMotorVoltage();

  ctre::phoenix6::StatusSignal<units::turn_t> positionRollerSig =
      algaeRollerMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> velocityRollerSig =
      algaeRollerMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::volt_t> voltageRollerSig =
      algaeRollerMotor.GetMotorVoltage();
  ctre::phoenix6::StatusSignal<units::ampere_t> torqueCurrentRollerSig =
      algaeRollerMotor.GetTorqueCurrent();

  ctre::phoenix6::controls::MotionMagicExpoVoltage algaePivotAngleSetter{0_rad};
  ctre::phoenix6::controls::VoltageOut algaePivotVoltageSetter{0_V};

  str::gains::radial::VoltRadialGainsHolder currentGains{
      consts::algae::gains::ALGAE_PIVOT_GAINS};
  units::volt_t currentKg{consts::algae::gains::kG};

  frc::LinearFilter<units::ampere_t> currentFilter =
      frc::LinearFilter<units::ampere_t>::MovingAverage(5);

  frc::Debouncer intakeSpikeDebouncer{.25_s, frc::Debouncer::kRising};

  frc::LinearSystem<1, 1, 1> rollerPlant{frc::LinearSystemId::FlywheelSystem(
      consts::algae::physical::ROLLER_MOTOR,
      consts::algae::physical::ROLLER_MOI,
      consts::algae::physical::ROLLER_GEARING)};

  frc::sim::FlywheelSim rollerSim{rollerPlant,
                                  consts::algae::physical::ROLLER_MOTOR};

  frc::sim::SingleJointedArmSim algaePivotSim{
      consts::algae::physical::PIVOT_MOTOR,
      consts::algae::physical::PIVOT_GEARING,
      consts::algae::physical::MOI,
      consts::algae::physical::ARM_LENGTH,
      consts::algae::physical::MIN_ANGLE,
      consts::algae::physical::MAX_ANGLE,
      true,
      -90_deg};

  frc2::sysid::SysIdRoutine algaePivotSysIdVoltage{
      frc2::sysid::Config{
          std::nullopt, 10_V, std::nullopt,
          [](frc::sysid::State state) {
            ctre::phoenix6::SignalLogger().WriteString(
                "SysIdAlgae_State",
                frc::sysid::SysIdRoutineLog::StateEnumToString(state));
          }},
      frc2::sysid::Mechanism{
          [this](units::volt_t voltsToSend) { SetPivotVoltage(voltsToSend); },
          [this](frc::sysid::SysIdRoutineLog* log) { LogAlgaePivotVolts(log); },
          this, "algaepivot"}};

  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("Algae")};
  nt::DoublePublisher currentAnglePub{
      nt->GetDoubleTopic("CurrentAlgaePivotAngle").Publish()};
  nt::DoublePublisher angleSetpointPub{
      nt->GetDoubleTopic("AlgaePivotAngleSetpoint").Publish()};
  nt::BooleanPublisher isAtSetpointPub{
      nt->GetBooleanTopic("IsAtRequestedPivotAngle").Publish()};
  nt::BooleanPublisher hasAlgaePub{nt->GetBooleanTopic("HasAlgae").Publish()};
  nt::DoublePublisher rollerTorquePub{
      nt->GetDoubleTopic("RollerTorqueCurrent").Publish()};
  nt::DoublePublisher filteredCurrentPub{
      nt->GetDoubleTopic("Filtered Current").Publish()};
  nt::BooleanSubscriber gotAlgaeSub{
      nt->GetBooleanTopic("SimGrabbingAlgae").Subscribe(false)};
  str::SuperstructureDisplay& display;
  std::string algaePivotAlertMsg{"Algae Pivot Motor Config"};
  frc::Alert configureAlgaePivotAlert{algaePivotAlertMsg,
                                      frc::Alert::AlertType::kError};
  std::string algaeRollerAlertMsg{"Algae Roller Motor Config"};
  frc::Alert configureAlgaeRollerAlert{algaeRollerAlertMsg,
                                       frc::Alert::AlertType::kError};
  std::string algaeOptiAlertMsg{"Algae Bus Optimization"};
  frc::Alert optiAlgaeAlert{algaeOptiAlertMsg, frc::Alert::AlertType::kError};
  std::string signalFrequencyAlertStr{"Algae Signal Frequency Set"};
  frc::Alert signalFrequencyAlert{signalFrequencyAlertStr,
                                  frc::Alert::AlertType::kError};
};
