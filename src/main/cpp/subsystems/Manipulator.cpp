// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Manipulator.h"
#include <frc/RobotBase.h>
#include <frc/DataLogManager.h>
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "frc2/command/button/Trigger.h"

Manipulator::Manipulator() {
  ConfigureMotors();
  ConfigureControlSignals();

  OptimizeBusSignals();

  nt->SetDefaultBoolean("SimBumpSwitch", false);
}

// This method will be called once per scheduler run
void Manipulator::Periodic() {
  ctre::phoenix::StatusCode status =
      ctre::phoenix6::BaseStatusSignal::RefreshAll(
          fwdLimitPressedSig, revLimitPressedSig, torqueCurrentSig,
          velocitySig);

  if (!status.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "Error updating manipulator signals! Error was: {}", status.GetName()));
  }

  previouslyHadAlgae = hasAlgae;
  hasAlgae = fwdLimitPressedSig.GetValue() ==
             ctre::phoenix6::signals::ForwardLimitValue::ClosedToGround;

  UpdateNTEntries();
}

void Manipulator::SimulationPeriodic() {
  rollerSim.SetForwardLimit(bumpSwitchSub.Get());
}

frc2::Trigger Manipulator::GotAlgae() {
  return frc2::Trigger{[this] {
    return (previouslyHadAlgae == false) && (previouslyHadAlgae != hasAlgae);
  }};
}

frc2::Trigger Manipulator::GotCoral() {
  return frc2::Trigger{[this] { return false; }};
}

bool Manipulator::HasCoral() {
  return hasCoral;
}
bool Manipulator::HasAlgae() {
  return hasAlgae;
}

void Manipulator::UpdateNTEntries() {
  hasAlgaePub.Set(hasAlgae);
  hasCoralPub.Set(hasCoral);
  gotAlgaePub.Set(GotAlgae().Get());
  gotCoralPub.Set(GotCoral().Get());
}

void Manipulator::SetVoltage(units::volt_t volts) {
  rollerVoltageSetter.WithEnableFOC(true)
      .WithIgnoreHardwareLimits(true)
      .WithOutput(volts);
}

void Manipulator::OptimizeBusSignals() {
  ctre::phoenix::StatusCode freqSetterStatus =
      ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
          consts::manip::BUS_UPDATE_FREQ, fwdLimitPressedSig,
          revLimitPressedSig, torqueCurrentSig, velocitySig);

  frc::DataLogManager::Log(
      fmt::format("Set bus signal frequenceies for manipulator. Result was: {}",
                  freqSetterStatus.GetName()));

  signalFrequencyAlert.Set(!freqSetterStatus.IsOK());

  ctre::phoenix::StatusCode optimizeManipResult =
      rollerMotor.OptimizeBusUtilization();
  frc::DataLogManager::Log(
      fmt::format("Optimized bus signals for manipulator motor. Result was: {}",
                  optimizeManipResult.GetName()));
  optiManipAlert.Set(!optimizeManipResult.IsOK());
}

void Manipulator::ConfigureMotors() {
  ctre::phoenix6::configs::TalonFXConfiguration config{};

  config.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  config.MotorOutput.Inverted =
      consts::manip::physical::INVERT_ROLLER
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  config.TorqueCurrent.PeakForwardTorqueCurrent =
      consts::manip::current_limits::STATOR_LIMIT;
  config.TorqueCurrent.PeakReverseTorqueCurrent =
      -consts::manip::current_limits::STATOR_LIMIT;

  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.CurrentLimits.SupplyCurrentLimit =
      consts::manip::current_limits::SUPPLY_LIMIT;

  if (frc::RobotBase::IsSimulation()) {
    config.MotorOutput.Inverted =
        ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  }

  ctre::phoenix::StatusCode configManipResult =
      rollerMotor.GetConfigurator().Apply(config);

  frc::DataLogManager::Log(
      fmt::format("Configured manipulator motor. Result was: {}",
                  configManipResult.GetName()));

  configureManipAlert.Set(!configManipResult.IsOK());
}

void Manipulator::ConfigureControlSignals() {
  rollerVoltageSetter.UpdateFreqHz = 0_Hz;
}