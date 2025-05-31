// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <memory>
#include <string>

#include <ctre/phoenix6/TalonFX.hpp>

#include "constants/L1Constants.h"
#include "ctre/phoenix6/SignalLogger.hpp"
#include "ctre/phoenix6/controls/MotionMagicExpoVoltage.hpp"
#include "ctre/phoenix6/controls/PositionVoltage.hpp"
#include "frc/Alert.h"
#include "frc/filter/LinearFilter.h"
#include "frc/simulation/FlywheelSim.h"
#include "frc/simulation/SingleJointedArmSim.h"
#include "frc/system/plant/LinearSystemId.h"
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

class L1 : public frc2::SubsystemBase {
 public:
  explicit L1(str::SuperstructureDisplay& display);  // Is this
                                                     // correct?
  void OptimizeBusSignals();
  void Periodic() override;
  void SimulationPeriodic() override;
  units::turn_t GetPivotAngle();
  void GoToAngle(units::turn_t newAngle);
  frc2::Trigger IsAtGoalAngle();
  void SetPivotVoltage(units::volt_t volts);
  frc2::CommandPtr Stow();
  frc2::CommandPtr Score();
  frc2::CommandPtr Deploy();
  frc2::CommandPtr Climb(std::function<units::volt_t()> volts);
  frc2::CommandPtr GoToAngleCmd(std::function<units::turn_t()> newAngle);
  frc2::CommandPtr TuneL1PID(std::function<bool()> isDone);

 private:
  void ConfigureMotors();
  void ConfigureControlSignals();
  void UpdateNTEntries();
  units::turns_per_second_t GetL1Vel();
  void SetL1Gains(str::gains::radial::VoltRadialGainsHolder newGains,
                  units::volt_t kg);

  ctre::phoenix6::hardware::TalonFX pivotMotor{
      consts::l1::can_ids::PIVOT_MOTOR};
  ctre::phoenix6::hardware::TalonFX rollerMotor{55};

  units::turn_t goalAngle = 0_rad;
  units::turn_t currentAngle = 0_rad;
  bool isAtGoalAngle = false;

  ctre::phoenix6::sim::TalonFXSimState& pivotMotorSim =
      pivotMotor.GetSimState();

  ctre::phoenix6::StatusSignal<units::turn_t> positionSig =
      pivotMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> velocitySig =
      pivotMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::volt_t> voltageSig =
      pivotMotor.GetMotorVoltage();

  ctre::phoenix6::controls::PositionVoltage pivotAngleSetter{0_rad};
  ctre::phoenix6::controls::VoltageOut pivotVoltageSetter{0_V};

  str::gains::radial::VoltRadialGainsHolder currentGains{
      consts::l1::gains::PIVOT_GAINS};
  units::volt_t currentKg{consts::l1::gains::kG};

  frc::sim::SingleJointedArmSim l1Sim{consts::l1::physical::CLIMBER_MOTOR,
                                      consts::l1::physical::GEARING,
                                      consts::l1::physical::MOI,
                                      consts::l1::physical::ARM_LENGTH,
                                      consts::l1::physical::MIN_ANGLE,
                                      consts::l1::physical::MAX_ANGLE,
                                      true,
                                      0_deg};

  str::SuperstructureDisplay& display;

  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("L1")};
  nt::DoublePublisher currentAnglePub{
      nt->GetDoubleTopic("CurrentL1Angle").Publish()};
  nt::DoublePublisher angleSetpointPub{
      nt->GetDoubleTopic("L1AngleSetpoint").Publish()};
  nt::BooleanPublisher isAtSetpointPub{
      nt->GetBooleanTopic("IsAtRequestedL1Angle").Publish()};
  std::string l1AlertMsg{"L1 Motor Config"};
  frc::Alert configureL1Alert{l1AlertMsg, frc::Alert::AlertType::kError};
  std::string l1OptiAlertMsg{"L1 Bus Optimization"};
  frc::Alert optiL1Alert{l1OptiAlertMsg, frc::Alert::AlertType::kError};
  std::string signalFrequencyAlertStr{"L1 Signal Frequency Set"};
  frc::Alert signalFrequencyAlert{signalFrequencyAlertStr,
                                  frc::Alert::AlertType::kError};
};
