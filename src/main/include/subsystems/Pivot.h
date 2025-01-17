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
#include "ctre/phoenix6/controls/MotionMagicExpoVoltage.hpp"
#include "ctre/phoenix6/sim/CANcoderSimState.hpp"
#include "frc/Alert.h"
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

class Pivot : public frc2::SubsystemBase {
 public:
  explicit Pivot(str::SuperstructureDisplay& display);
  void OptimizeBusSignals();

  void Periodic() override;
  void SimulationPeriodic() override;
  units::radian_t GetAngle();
  void GoToAngle(units::radian_t newAngle);
  frc2::Trigger IsAtGoalAngle();
  void SetVoltage(units::volt_t volts);
  frc2::CommandPtr GoToAngleCmd(std::function<units::radian_t()> newAngle);
  frc2::CommandPtr SysIdPivotQuasistaticVoltage(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdPivotDynamicVoltage(frc2::sysid::Direction dir);
  frc2::CommandPtr TunePivotPID(std::function<bool()> isDone);

 private:
  void ConfigureMotors();
  void ConfigureControlSignals();
  void UpdateNTEntries();
  units::radians_per_second_t GetPivotVel();
  void SetPivotGains(str::gains::radial::VoltRadialGainsHolder newGains,
                     units::volt_t kg);
  units::radian_t ConvertEncPosToAngle(units::turn_t rots);
  units::turn_t ConvertAngleToEncPos(units::radian angle);
  units::radians_per_second_t ConvertEncVelToAngleVel(
      units::turns_per_second_t radialVel);
  units::turns_per_second_t ConvertAngleVelToEncVel(
      units::radians_per_second_t vel);
  void LogPivotVolts(frc::sysid::SysIdRoutineLog* log);

  ctre::phoenix6::hardware::TalonFX pivotMotor{
      consts::pivot::can_ids::PIVOT_MOTOR};

  units::radian_t goalAngle = 0_rad;
  units::radian_t currentAngle = 0_rad;
  bool isAtGoalAngle = false;

  ctre::phoenix6::sim::TalonFXSimState& pivotMotorSim =
      pivotMotor.GetSimState();

  ctre::phoenix6::StatusSignal<units::turn_t> positionSig =
      pivotMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> velocitySig =
      pivotMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::volt_t> voltageSig =
      pivotMotor.GetMotorVoltage();

  ctre::phoenix6::controls::MotionMagicExpoVoltage pivotAngleSetter{0_rad};
  ctre::phoenix6::controls::VoltageOut pivotVoltageSetter{0_V};

  str::gains::radial::VoltRadialGainsHolder currentGains{
      consts::pivot::gains::PIVOT_GAINS};
  units::volt_t currentKg{consts::pivot::gains::kG};

  //   frc::sim::SingleJointedArmSim pivotSim{
  //       consts::pivot::physical::MOTOR,
  //       consts::pivot::physical::GEARING,
  //       consts::pivot::physical::CARRIAGE_MASS,
  //       consts::pivot::physical::PULLEY_DIAM / 2,
  //       0_m,
  //       consts::pivot::physical::EXTENDED_HEIGHT,
  //       true,
  //       0_m,
  //       {0.00}};

  frc2::sysid::SysIdRoutine pivotSysIdVoltage{
      frc2::sysid::Config{
          std::nullopt, 10_V, std::nullopt,
          [](frc::sysid::State state) {
            ctre::phoenix6::SignalLogger().WriteString(
                "SysIdPivot_State",
                frc::sysid::SysIdRoutineLog::StateEnumToString(state));
          }},
      frc2::sysid::Mechanism{
          [this](units::volt_t voltsToSend) { SetVoltage(voltsToSend); },
          [this](frc::sysid::SysIdRoutineLog* log) { LogPivotVolts(log); },
          this, "pivot"}};

  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("Pivot")};
  nt::DoublePublisher currentAnglePub{
      nt->GetDoubleTopic("CurrentAngle").Publish()};
  nt::DoublePublisher angleSetpointPub{
      nt->GetDoubleTopic("AngleSetpoint").Publish()};
  nt::BooleanPublisher isAtSetpointPub{
      nt->GetBooleanTopic("IsAtRequestedAngle").Publish()};
  str::SuperstructureDisplay& display;
  std::string pivotAlertMsg{"Pivot Motor Config"};
  frc::Alert configurePivotAlert{pivotAlertMsg, frc::Alert::AlertType::kError};
  std::string pivotOptiAlertMsg{"Pivot Bus Optimization"};
  frc::Alert optiPivotAlert{pivotOptiAlertMsg, frc::Alert::AlertType::kError};
  std::string signalFrequencyAlertStr{"Pivot Signal Frequency Set"};
  frc::Alert signalFrequencyAlert{signalFrequencyAlertStr,
                                  frc::Alert::AlertType::kError};
};
