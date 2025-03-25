// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <memory>
#include <string>

#include <ctre/phoenix6/TalonFX.hpp>

#include "constants/ClimberConstants.h"
#include "ctre/phoenix6/SignalLogger.hpp"
#include "ctre/phoenix6/controls/MotionMagicExpoVoltage.hpp"
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
#include "frc/Servo.h"

class Climber : public frc2::SubsystemBase {
 public:
  explicit Climber(str::SuperstructureDisplay& display);  // Is this
                                                          // correct?
  void OptimizeBusSignals();
  void Periodic() override;
  void SimulationPeriodic() override;
  units::turn_t GetClimberAngle();
  void GoToAngle(units::turn_t newAngle);
  frc2::Trigger IsAtGoalAngle();
  void SetClimberVoltage(units::volt_t volts);
  frc2::CommandPtr Stow();
  frc2::CommandPtr Deploy();
  frc2::CommandPtr Climb(std::function<units::volt_t()> volts);
  frc2::CommandPtr GoToAngleCmd(std::function<units::turn_t()> newAngle);
  frc2::CommandPtr TuneClimberPID(std::function<bool()> isDone);
  frc2::CommandPtr Lock();
  frc2::CommandPtr Unlock();

 private:
  void ConfigureMotors();
  void ConfigureControlSignals();
  void UpdateNTEntries();
  units::turns_per_second_t GetClimberVel();
  void SetClimberGains(str::gains::radial::VoltRadialGainsHolder newGains,
                       units::volt_t kg);

  ctre::phoenix6::hardware::TalonFX climberMotor{
      consts::climber::can_ids::CLIMBER_MOTOR};

  frc::Servo climberLock{1};

  units::turn_t goalAngle = 0_rad;
  units::turn_t currentAngle = 0_rad;
  bool isAtGoalAngle = false;

  ctre::phoenix6::sim::TalonFXSimState& climberMotorSim =
      climberMotor.GetSimState();

  ctre::phoenix6::StatusSignal<units::turn_t> positionSig =
      climberMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> velocitySig =
      climberMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::volt_t> voltageSig =
      climberMotor.GetMotorVoltage();

  ctre::phoenix6::controls::MotionMagicExpoVoltage climberAngleSetter{0_rad};
  ctre::phoenix6::controls::VoltageOut climberVoltageSetter{0_V};

  str::gains::radial::VoltRadialGainsHolder currentGains{
      consts::climber::gains::CLIMBER_GAINS};
  units::volt_t currentKg{consts::climber::gains::kG};

  frc::sim::SingleJointedArmSim climberSim{
      consts::climber::physical::CLIMBER_MOTOR,
      consts::climber::physical::GEARING,
      consts::climber::physical::MOI,
      consts::climber::physical::ARM_LENGTH,
      consts::climber::physical::MIN_ANGLE,
      consts::climber::physical::MAX_ANGLE,
      true,
      0_deg};

  str::SuperstructureDisplay& display;

  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("Climber")};
  nt::DoublePublisher currentAnglePub{
      nt->GetDoubleTopic("CurrentClimberAngle").Publish()};
  nt::DoublePublisher angleSetpointPub{
      nt->GetDoubleTopic("ClimberAngleSetpoint").Publish()};
  nt::BooleanPublisher isAtSetpointPub{
      nt->GetBooleanTopic("IsAtRequestedClimberAngle").Publish()};
  std::string climberAlertMsg{"Climber Motor Config"};
  frc::Alert configureClimberAlert{climberAlertMsg,
                                   frc::Alert::AlertType::kError};
  std::string climberOptiAlertMsg{"Climber Bus Optimization"};
  frc::Alert optiClimberAlert{climberOptiAlertMsg,
                              frc::Alert::AlertType::kError};
  std::string signalFrequencyAlertStr{"Climber Signal Frequency Set"};
  frc::Alert signalFrequencyAlert{signalFrequencyAlertStr,
                                  frc::Alert::AlertType::kError};
};
