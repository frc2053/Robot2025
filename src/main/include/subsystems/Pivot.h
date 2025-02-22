// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <memory>
#include <string>

#include <ctre/phoenix6/TalonFX.hpp>

#include "constants/PivotConstants.h"
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/SignalLogger.hpp"
#include "ctre/phoenix6/sim/CANcoderSimState.hpp"
#include "frc/Alert.h"
#include "frc/controller/ArmFeedforward.h"
#include "frc/controller/PIDController.h"
#include "frc/simulation/SingleJointedArmSim.h"
#include "frc/sysid/SysIdRoutineLog.h"
#include "frc/trajectory/ExponentialProfile.h"
#include "frc/trajectory/TrapezoidProfile.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/button/Trigger.h"
#include "frc2/command/sysid/SysIdRoutine.h"
#include "networktables/BooleanTopic.h"
#include "networktables/DoubleTopic.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/StringTopic.h"
#include "str/GainTypes.h"
#include "str/SuperstructureDisplay.h"
#include "units/angle.h"
#include "units/angular_acceleration.h"
#include "units/angular_velocity.h"
#include "units/voltage.h"

class Pivot : public frc2::SubsystemBase {
 public:
  explicit Pivot(str::SuperstructureDisplay& display);
  void OptimizeBusSignals();

  void Periodic() override;
  void SimulationPeriodic() override;
  units::turn_t GetAngle();
  void GoToAngle(units::turn_t newAngle, bool isIntermediateState);
  frc2::Trigger IsAtGoalAngle();
  frc2::Trigger IsClearOfFunnel();
  void SetVoltage(units::volt_t volts);
  frc2::CommandPtr Coast();
  frc2::CommandPtr GoToAngleCmd(std::function<units::turn_t()> newAngle);
  frc2::CommandPtr GoToAngleCmd(std::function<units::turn_t()> newAngle,
                                std::function<bool()> isIntermediateState);
  frc2::CommandPtr SysIdPivotQuasistaticVoltage(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdPivotDynamicVoltage(frc2::sysid::Direction dir);
  frc2::CommandPtr TunePivotPID(std::function<bool()> isDone);
  void SetToStartingPosition();

 private:
  void ConfigureMotors();
  void ConfigureControlSignals();
  void UpdateNTEntries();
  units::turns_per_second_t GetPivotVel();
  void SetPivotGains(str::gains::radial::VoltRadialGainsHolder newGains,
                     units::volt_t kg, units::turns_per_second_t newMaxVel,
                     units::turns_per_second_squared_t newMaxAccel);
  void LogPivotVolts(frc::sysid::SysIdRoutineLog* log);

  ctre::phoenix6::hardware::TalonFX pivotMotor{
      consts::pivot::can_ids::PIVOT_MOTOR};

  ctre::phoenix6::hardware::CANcoder pivotEncoder{
      consts::pivot::can_ids::PIVOT_ENC};

  units::turn_t currentAngle = 0_rad;
  bool isAtGoalAngle = false;

  ctre::phoenix6::sim::TalonFXSimState& pivotMotorSim =
      pivotMotor.GetSimState();
  ctre::phoenix6::sim::CANcoderSimState& encoderSim =
      pivotEncoder.GetSimState();

  ctre::phoenix6::StatusSignal<units::turn_t> positionSig =
      pivotMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> velocitySig =
      pivotMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::volt_t> voltageSig =
      pivotMotor.GetMotorVoltage();

  ctre::phoenix6::controls::VoltageOut pivotVoltageSetter{0_V};
  ctre::phoenix6::controls::CoastOut coastSetter{};

  str::gains::radial::VoltRadialGainsHolder currentGains{
      consts::pivot::gains::PIVOT_GAINS};
  units::volt_t currentKg{consts::pivot::gains::kG};

  frc::ArmFeedforward ff{currentGains.kS, currentKg, currentGains.kV,
                         currentGains.kA};

  units::degrees_per_second_t maxProfVel = 720_deg_per_s;
  units::degrees_per_second_squared_t maxProfAccel = 720_deg_per_s_sq;

  frc::TrapezoidProfile<units::turns> trapProf{
      frc::TrapezoidProfile<units::turns>::Constraints{maxProfVel,
                                                       maxProfAccel}};
  frc::TrapezoidProfile<units::turns>::State trapSetpoint{};
  frc::TrapezoidProfile<units::turns>::State trapGoal{};

  frc::PIDController pivotPid{currentGains.kP.value(), currentGains.kI.value(),
                              currentGains.kD.value()};

  units::volt_t ffToSend{0_V};
  double pidOutput{0};
  bool isCharacterizing{false};

  frc::sim::SingleJointedArmSim pivotSim{consts::pivot::physical::MOTOR,
                                         consts::pivot::physical::GEARING,
                                         consts::pivot::physical::MOI,
                                         consts::pivot::physical::ARM_LENGTH,
                                         consts::pivot::physical::MIN_ANGLE,
                                         consts::pivot::physical::MAX_ANGLE,
                                         true,
                                         0_rad};

  frc2::sysid::SysIdRoutine pivotSysIdVoltage{
      frc2::sysid::Config{
          (1_V / 1_s), 5_V, std::nullopt,
          [this](frc::sysid::State state) {
            ctre::phoenix6::SignalLogger::WriteString(
                "SysIdPivot-State",
                frc::sysid::SysIdRoutineLog::StateEnumToString(state));
            pivotSysIdStatePub.Set(
                frc::sysid::SysIdRoutineLog::StateEnumToString(state));
          }},
      frc2::sysid::Mechanism{
          [this](units::volt_t voltsToSend) { SetVoltage(voltsToSend); },
          [this](frc::sysid::SysIdRoutineLog* log) { LogPivotVolts(log); },
          this, "pivot"}};

  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("Pivot")};

  nt::StringPublisher pivotSysIdStatePub{
      nt->GetStringTopic("PivotSysIdState").Publish()};
  nt::DoublePublisher appliedVoltagePub{
      nt->GetDoubleTopic("AppliedVoltage").Publish()};
  nt::DoublePublisher currentAnglePub{
      nt->GetDoubleTopic("CurrentAngle").Publish()};
  nt::DoublePublisher currentVelPub{
      nt->GetDoubleTopic("CurrentAngularVelocity").Publish()};
  nt::DoublePublisher angularPosSetpointPub{
      nt->GetDoubleTopic("AngularPosSetpoint").Publish()};
  nt::DoublePublisher angularVelSetpointPub{
      nt->GetDoubleTopic("AngularVelSetpoint").Publish()};
  nt::DoublePublisher angularGoalPub{
      nt->GetDoubleTopic("AngularGoal").Publish()};
  nt::BooleanPublisher isAtSetpointPub{
      nt->GetBooleanTopic("IsAtRequestedAngle").Publish()};
  str::SuperstructureDisplay& display;
  std::string pivotEncoderAlertMsg{"Pivot Encoder Config"};
  frc::Alert configureEncoderAlert{pivotEncoderAlertMsg,
                                   frc::Alert::AlertType::kError};
  std::string pivotAlertMsg{"Pivot Motor Config"};
  frc::Alert configurePivotAlert{pivotAlertMsg, frc::Alert::AlertType::kError};
  std::string pivotOptiAlertMsg{"Pivot Bus Optimization"};
  frc::Alert optiPivotAlert{pivotOptiAlertMsg, frc::Alert::AlertType::kError};
  std::string signalFrequencyAlertStr{"Pivot Signal Frequency Set"};
  frc::Alert signalFrequencyAlert{signalFrequencyAlertStr,
                                  frc::Alert::AlertType::kError};
};
