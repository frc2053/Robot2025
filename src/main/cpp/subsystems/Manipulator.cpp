// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/Manipulator.h"

#include <frc/DataLogManager.h>
#include <frc/RobotBase.h>

#include "constants/ManipulatorConstants.h"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "frc/RobotController.h"
#include "frc2/command/Commands.h"
#include "frc2/command/button/Trigger.h"

Manipulator::Manipulator(str::SuperstructureDisplay& display)
    : display{display} {
  ConfigureMotors();
  ConfigureControlSignals();

  OptimizeBusSignals();

  nt->SetDefaultBoolean("SimBumpSwitch", false);
  nt->SetDefaultBoolean("SimGrabbingCoral", false);
  nt->SetDefaultBoolean("SimDroppingCoral", false);
}

void Manipulator::OverrideHasCoral(bool overrideCoral) {
  hasCoral = overrideCoral;
}

frc2::CommandPtr Manipulator::PoopPiece() {
  return frc2::cmd::Run([this] { Poop(); });
}

// TODO: Might be able to detect wheel speed increase to get rid of time
frc2::CommandPtr Manipulator::PoopPiece(
    std::function<units::second_t()> timeToPoop) {
  return PoopPiece().WithTimeout(timeToPoop()).FinallyDo([this] { Stop(); });
}

frc2::CommandPtr Manipulator::SuckUntilAlgae() {
  return frc2::cmd::Run([this] { Suck(); }, {this})
      .Until([this] { return HasAlgae(); })
      .AndThen(HoldCmd());
}

frc2::CommandPtr Manipulator::AlgaeAuto() {
  return frc2::cmd::RunOnce([this] { Suck(); }, {this});
}

frc2::CommandPtr Manipulator::SuckUntilCoral() {
  return frc2::cmd::Run(
             [this] {
               Suck();
               fmt::print("Sucking coral!\n");
             },
             {this})
      .Until([this] { return HasCoral(); });
}

frc2::CommandPtr Manipulator::StopCmd() {
  return frc2::cmd::RunOnce([this] { Stop(); }, {this});
}

frc2::CommandPtr Manipulator::HoldCoralCmd() {
  return frc2::cmd::RunOnce([this] { SetVoltage(-.5_V); }, {this});
}

frc2::CommandPtr Manipulator::HoldCmd() {
  return frc2::cmd::Run([this] { SetVoltage(-2_V); }, {this});
}

// This method will be called once per scheduler run
void Manipulator::Periodic() {
  ctre::phoenix::StatusCode status =
      ctre::phoenix6::BaseStatusSignal::RefreshAll(
          fwdLimitPressedSig, revLimitPressedSig, torqueCurrentSig, velocitySig,
          voltageSig);

  if (!status.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "Error updating manipulator signals! Error was: {}", status.GetName()));
  }

  previouslyHadCoral = hasCoral;
  previouslyHadAlgae = hasAlgae;
  hasAlgae = revLimitPressedSig.GetValue() ==
             ctre::phoenix6::signals::ForwardLimitValue::ClosedToGround;

  currentVoltage = voltageSig.GetValue();
  torqueCurrent = torqueCurrentSig.GetValue();
  currentVelocity = velocitySig.GetValue();

  if (frc::RobotBase::IsSimulation()) {
    if (fakeCoralSuck) {
      torqueCurrent = -400_A;
    }
    if (fakeCoralDrop) {
      torqueCurrent = 400_A;
    }
  }

  hasCoral = (gotCoral.Get() || previouslyHadCoral) && !droppedCoral.Get();

  UpdateNTEntries();
}

void Manipulator::Poop() {
  SetVoltage(consts::manip::gains::POOP_VOLTS);
}

void Manipulator::Suck() {
  SetVoltage(-consts::manip::gains::SUCK_VOLTS);
}

void Manipulator::Stop() {
  SetVoltage(0_V);
}

void Manipulator::SimulationPeriodic() {
  fakeCoralSuck = gotCorralSub.Get();
  fakeCoralDrop = droppedCoralSub.Get();
  rollerSim.SetReverseLimit(bumpSwitchSub.Get());
  rollerSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

  coralWheelSim.SetInputVoltage(rollerSim.GetMotorVoltage());
  coralWheelSim.Update(20_ms);

  rollerSim.SetRotorVelocity(coralWheelSim.GetAngularVelocity() *
                             consts::manip::physical::CORAL_REDUCTION);
}

frc2::Trigger Manipulator::GotAlgae() {
  return frc2::Trigger{[this] {
    return (previouslyHadAlgae == false) && (previouslyHadAlgae != hasAlgae);
  }};
}

frc2::Trigger Manipulator::DroppedCoral() {
  return frc2::Trigger{[this] {
           return (torqueCurrent >
                   consts::manip::gains::DROPPED_GAME_PIECE_CURRENT) &&
                  (previouslyHadCoral == true);
         }}
      .Debounce(consts::manip::gains::CORAL_DEBOUNCE_TIME);
}

frc2::Trigger Manipulator::GotCoralFR() {
  return frc2::Trigger{[this] { return hasCoral; }};
}

frc2::Trigger Manipulator::GotCoral() {
  return frc2::Trigger{[this] {
           return (torqueCurrent <
                   consts::manip::gains::GOT_GAME_PIECE_CURRENT) &&
                  (previouslyHadCoral == false) && tryingForCoral;
         }}
      .Debounce(consts::manip::gains::CORAL_DEBOUNCE_TIME);
}

void Manipulator::SetTryingForCoral(bool newValue) {
  tryingForCoral = newValue;
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
  gotAlgaePub.Set(gotAlgae.Get());
  gotCoralPub.Set(gotCoral.Get());
  voltagePub.Set(currentVoltage.value());
  velocityPub.Set(currentVelocity.value());
  torqueCurrentPub.Set(torqueCurrent.value());
  commandedVoltagePub.Set(commandedVoltage.value());
  droppedCoralPub.Set(droppedCoral.Get());
  tryingForCoralPub.Set(tryingForCoral);
  display.GamePieceSet(hasCoral, hasAlgae);
}

void Manipulator::SetVoltage(units::volt_t volts) {
  commandedVoltage = volts;
  rollerMotor.SetControl(rollerVoltageSetter.WithEnableFOC(true)
                             .WithIgnoreHardwareLimits(true)
                             .WithOutput(commandedVoltage));
}

void Manipulator::OptimizeBusSignals() {
  ctre::phoenix::StatusCode freqSetterStatus =
      ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
          consts::manip::BUS_UPDATE_FREQ, fwdLimitPressedSig,
          revLimitPressedSig, torqueCurrentSig, velocitySig, voltageSig);

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

  config.CurrentLimits.StatorCurrentLimitEnable = true;
  config.CurrentLimits.StatorCurrentLimit =
      consts::manip::current_limits::STATOR_LIMIT;

  config.OpenLoopRamps.VoltageOpenLoopRampPeriod = .05_s;

  config.HardwareLimitSwitch.ForwardLimitEnable = false;
  config.HardwareLimitSwitch.ReverseLimitEnable = false;

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
  rollerVoltageSetter.UpdateFreqHz = 100_Hz;
}
