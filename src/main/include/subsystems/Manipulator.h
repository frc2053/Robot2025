// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <memory>
#include <string>

#include "constants/ManipulatorConstants.h"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "ctre/phoenix6/sim/TalonFXSimState.hpp"
#include "frc/Alert.h"
#include "frc/filter/LinearFilter.h"
#include "frc/simulation/FlywheelSim.h"
#include "frc/system/plant/LinearSystemId.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/button/Trigger.h"
#include "networktables/BooleanTopic.h"
#include "networktables/DoubleTopic.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "str/SuperstructureDisplay.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/time.h"

class Manipulator : public frc2::SubsystemBase {
 public:
  explicit Manipulator(str::SuperstructureDisplay& display);
  void OptimizeBusSignals();
  void OverrideHasCoral(bool hasCoral);

  void Periodic() override;
  void SimulationPeriodic() override;
  frc2::Trigger GotAlgae();
  frc2::Trigger GotCoral();
  frc2::Trigger DroppedCoral();
  bool HasCoral();
  bool HasAlgae();
  void Poop();
  void Suck();
  void Stop();
  void SetTryingForCoral(bool newValue);

  frc2::CommandPtr PoopPiece();
  // TODO: Might be able to detect wheel speed increase to get rid of time
  frc2::CommandPtr PoopPiece(std::function<units::second_t()> timeToPoop);
  frc2::CommandPtr SuckUntilAlgae();
  frc2::CommandPtr SuckUntilCoral();
  frc2::CommandPtr StopCmd();
  frc2::CommandPtr HoldCmd();
  frc2::CommandPtr HoldCoralCmd();
  frc2::Trigger GotCoralFR();
  frc2::CommandPtr AlgaeAuto();

 private:
  void ConfigureMotors();
  void ConfigureControlSignals();
  void UpdateNTEntries();
  void SetVoltage(units::volt_t volts);

  bool hasAlgae{false};
  bool hasCoral{false};
  bool previouslyHadAlgae{false};
  bool previouslyHadCoral{false};
  bool fakeCoralDrop{false};
  bool fakeCoralSuck{false};
  bool tryingForCoral{true};

  units::volt_t currentVoltage{0_V};
  units::volt_t commandedVoltage{0_V};
  units::ampere_t torqueCurrent{0_A};
  units::revolutions_per_minute_t currentVelocity{0_rpm};

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
  ctre::phoenix6::StatusSignal<units::volt_t> voltageSig =
      rollerMotor.GetMotorVoltage();

  ctre::phoenix6::controls::VoltageOut rollerVoltageSetter{0_V};

  frc::LinearSystem<1, 1, 1> coralWheelPlant{
      frc::LinearSystemId::FlywheelSystem(
          consts::manip::physical::MOTOR, consts::manip::physical::MOI,
          consts::manip::physical::CORAL_REDUCTION)};

  frc::sim::FlywheelSim coralWheelSim{coralWheelPlant,
                                      consts::manip::physical::MOTOR};

  frc::LinearFilter<units::ampere_t> currentFilter =
      frc::LinearFilter<units::ampere_t>::MovingAverage(3);

  frc2::Trigger gotCoral{GotCoral()};
  frc2::Trigger gotAlgae{GotAlgae()};
  frc2::Trigger droppedCoral{DroppedCoral()};

  str::SuperstructureDisplay& display;
  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("Manipulator")};
  nt::DoublePublisher voltagePub{nt->GetDoubleTopic("Voltage").Publish()};
  nt::DoublePublisher commandedVoltagePub{
      nt->GetDoubleTopic("Commanded Voltage").Publish()};
  nt::DoublePublisher velocityPub{nt->GetDoubleTopic("Velocity").Publish()};
  nt::DoublePublisher torqueCurrentPub{
      nt->GetDoubleTopic("TorqueCurrent").Publish()};
  nt::BooleanSubscriber bumpSwitchSub{
      nt->GetBooleanTopic("SimBumpSwitch").Subscribe(false)};
  nt::BooleanSubscriber gotCorralSub{
      nt->GetBooleanTopic("SimGrabbingCoral").Subscribe(false)};
  nt::BooleanSubscriber droppedCoralSub{
      nt->GetBooleanTopic("SimDroppingCoral").Subscribe(false)};
  nt::BooleanPublisher hasAlgaePub{nt->GetBooleanTopic("HasAlgae").Publish()};
  nt::BooleanPublisher hasCoralPub{nt->GetBooleanTopic("HasCoral").Publish()};
  nt::BooleanPublisher gotAlgaePub{nt->GetBooleanTopic("GotAlgae").Publish()};
  nt::BooleanPublisher gotCoralPub{nt->GetBooleanTopic("GotCoral").Publish()};
  nt::BooleanPublisher tryingForCoralPub{
      nt->GetBooleanTopic("TryingForCoral").Publish()};
  nt::BooleanPublisher droppedCoralPub{
      nt->GetBooleanTopic("DroppedCoral").Publish()};

  std::string manipAlertMsg{"Manipulator Motor Config"};
  frc::Alert configureManipAlert{manipAlertMsg, frc::Alert::AlertType::kError};
  std::string manipOptiAlertMsg{"Manipulator Bus Optimization"};
  frc::Alert optiManipAlert{manipOptiAlertMsg, frc::Alert::AlertType::kError};
  std::string signalFrequencyAlertStr{"Manipulator Signal Frequency Set"};
  frc::Alert signalFrequencyAlert{signalFrequencyAlertStr,
                                  frc::Alert::AlertType::kError};
};
