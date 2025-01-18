// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "ctre/phoenix6/sim/TalonFXSimState.hpp"
#include "frc/Alert.h"
#include "frc2/command/button/Trigger.h"
#include "networktables/BooleanTopic.h"
#include "networktables/DoubleTopic.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "constants/ManipulatorConstants.h"
#include "units/angular_velocity.h"
#include "units/current.h"

class Manipulator : public frc2::SubsystemBase {
 public:
  Manipulator();
  void OptimizeBusSignals();

  void Periodic() override;
  void SimulationPeriodic() override;
  frc2::Trigger GotAlgae();
  frc2::Trigger GotCoral();
  bool HasCoral();
  bool HasAlgae();

 private:
  void ConfigureMotors();
  void ConfigureControlSignals();
  void UpdateNTEntries();
  void SetVoltage(units::volt_t volts);

  bool hasAlgae{false};
  bool hasCoral{false};
  bool previouslyHadAlgae{false};

  ctre::phoenix6::hardware::TalonFX rollerMotor{
      consts::manip::can_ids::ROLLER_MOTOR};

  ctre::phoenix6::sim::TalonFXSimState rollerSim{rollerMotor};

  ctre::phoenix6::StatusSignal<ctre::phoenix6::signals::ForwardLimitValue>
      fwdLimitPressedSig = rollerMotor.GetForwardLimit();
  ctre::phoenix6::StatusSignal<ctre::phoenix6::signals::ReverseLimitValue>
      revLimitPressedSig = rollerMotor.GetReverseLimit();
  ctre::phoenix6::StatusSignal<units::ampere_t> torqueCurrentSig =
      rollerMotor.GetTorqueCurrent();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> velocitySig =
      rollerMotor.GetVelocity();

  ctre::phoenix6::controls::VoltageOut rollerVoltageSetter{0_V};

  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("Manipulator")};
  nt::DoublePublisher velocityPub{nt->GetDoubleTopic("Velocity").Publish()};
  nt::DoublePublisher torqueCurrentPub{
      nt->GetDoubleTopic("TorqueCurrent").Publish()};
  nt::BooleanSubscriber bumpSwitchSub{
      nt->GetBooleanTopic("SimBumpSwitch").Subscribe(false)};
  nt::BooleanPublisher hasAlgaePub{nt->GetBooleanTopic("HasAlgae").Publish()};
  nt::BooleanPublisher hasCoralPub{nt->GetBooleanTopic("HasCoral").Publish()};
  nt::BooleanPublisher gotAlgaePub{nt->GetBooleanTopic("GotAlgae").Publish()};
  nt::BooleanPublisher gotCoralPub{nt->GetBooleanTopic("GotCoral").Publish()};

  std::string manipAlertMsg{"Manipulator Motor Config"};
  frc::Alert configureManipAlert{manipAlertMsg, frc::Alert::AlertType::kError};
  std::string manipOptiAlertMsg{"Manipulator Bus Optimization"};
  frc::Alert optiManipAlert{manipOptiAlertMsg, frc::Alert::AlertType::kError};
  std::string signalFrequencyAlertStr{"Manipulator Signal Frequency Set"};
  frc::Alert signalFrequencyAlert{signalFrequencyAlertStr,
                                  frc::Alert::AlertType::kError};
};
