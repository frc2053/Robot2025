// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/Coordinator.h"

#include "constants/Presets.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "subsystems/Manipulator.h"

Coordinator::Coordinator(Elevator& elevator, Pivot& pivot,
                         Manipulator& manipulator, Climber& climber)
    : elev{elevator}, piv{pivot}, manip{manipulator}, climb{climber} {}

frc2::CommandPtr Coordinator::GoToL1() {
  return frc2::cmd::Either(GoToAlgaeProcess(), GoToAlgaeProcess(),
                           [this] { return manip.HasCoral(); });
}

frc2::CommandPtr Coordinator::GoToL2() {
  return frc2::cmd::Either(GoToL2Coral(), GoToL2Algae(),
                           [this] { return manip.HasCoral(); });
}

frc2::CommandPtr Coordinator::GoToL3() {
  return frc2::cmd::Either(GoToL3Coral(), GoToL3Algae(),
                           [this] { return manip.HasCoral(); });
}

frc2::CommandPtr Coordinator::GoToL4() {
  return frc2::cmd::Either(GoToL4Coral(), GoToNet(),
                           [this] { return manip.HasCoral(); });
}

frc2::CommandPtr Coordinator::GetOutOfStartingConfig() {
  return frc2::cmd::Sequence(
      climb.Stow(),
      frc2::cmd::RunOnce([this] { manip.SetTryingForCoral(false); }),
      piv.GoToAngleCmd([] { return presets::wrist::outofstarting; }),
      frc2::cmd::Print("Done with pivot!\n"),
      elev.GoToHeightCmd([] { return presets::elev::outofstarting; }),
      frc2::cmd::Print("Done with elevator!\n"),
      frc2::cmd::Print("Done with Out of starting!\n"));
}

frc2::CommandPtr Coordinator::GoToAlgaeHold() {
  return frc2::cmd::Parallel(
      manip.HoldCmd(),
      frc2::cmd::RunOnce([this] { manip.SetTryingForCoral(false); }),
      elev.GoToHeightCmd([] { return presets::elev::algae::hold; }),
      piv.GoToAngleCmd([] { return presets::wrist::algaeHold; }));
}

frc2::CommandPtr Coordinator::GoToCoralPrime() {
  return frc2::cmd::Parallel(
      frc2::cmd::Print("I have a coral, going to primed position"),
      piv.GoToAngleCmd([] { return presets::wrist::primed; }),
      elev.GoToHeightCmd([] { return presets::elev::coral::loading; }));
}

frc2::CommandPtr Coordinator::GoToLoading(bool override) {
  return frc2::cmd::Either(
      frc2::cmd::Parallel(
          manip.HoldCoralCmd(),
          frc2::cmd::Print("I have a coral, going to primed position"),
          piv.GoToAngleCmd([] { return presets::wrist::primed; }),
          elev.GoToHeightCmd([] { return presets::elev::coral::loading; })),
      frc2::cmd::Parallel(
          manip.StopCmd(),
          frc2::cmd::Print("I dont have a coral, going to loading position"),
          frc2::cmd::RunOnce([this] { manip.SetTryingForCoral(true); }),
          piv.GoToAngleCmd([] { return presets::wrist::coral::loading; }),
          elev.GoToHeightCmd([] { return presets::elev::coral::loading; })),
      [this, override] {
        if (override) {
          return false;
        }
        bool hasCoral = manip.HasCoral();
        fmt::print("Do I have a coral: {}\n", hasCoral);
        return hasCoral;
      });
}

// frc2::CommandPtr Coordinator::GoToLoading() { //Original Version
//   return frc2::cmd::Parallel(
//       elev.GoToHeightCmd([] { return presets::elev::coral::loading; }),
//       piv.GoToAngleCmd([] { return presets::wrist::coral::loading; }));
// }

frc2::CommandPtr Coordinator::WaitForPriming() {
  return frc2::cmd::WaitUntil([this] {
    bool clear = piv.IsClearOfFunnel().Get();
    fmt::print("Clear of funnel: {}\n", clear);
    return clear;
  });
}

frc2::CommandPtr Coordinator::PrimeCoral(
    std::function<units::radian_t()> finalAngle) {
  return frc2::cmd::Sequence(
      frc2::cmd::Print("Prime Coral"),
      frc2::cmd::Race(WaitForPriming(), piv.GoToAngleCmd(finalAngle)),
      frc2::cmd::Either(
          frc2::cmd::None(),
          elev.GoToHeightCmd([this] { return presets::elev::clearOfChassis; }),
          [this] {
            bool isClear = elev.GetHeight() >= presets::elev::clearOfChassis;
            fmt::print("isClear, should skip me!: {}\n", isClear);
            return isClear;
          }));
}

frc2::CommandPtr Coordinator::GoToL1Coral() {
  return frc2::cmd::Sequence(
      frc2::cmd::WaitUntil([this] { return piv.IsClearOfFunnel().Get(); }),
      frc2::cmd::RunOnce([this] { manip.SetTryingForCoral(false); }),
      elev.GoToHeightCmd([] { return presets::elev::coral::l1; }),
      piv.GoToAngleCmd([] { return presets::wrist::coral::l1; }));
}

frc2::CommandPtr Coordinator::GoToAlgaeProcess() {
  return frc2::cmd::Parallel(
      frc2::cmd::RunOnce([this] { manip.SetTryingForCoral(false); }),
      elev.GoToHeightCmd([] { return presets::elev::algae::processor; }),
      piv.GoToAngleCmd([] { return presets::wrist::algaeProcess; }));
}

frc2::CommandPtr Coordinator::GoToL2Coral() {
  return frc2::cmd::Parallel(
      frc2::cmd::RunOnce([this] { manip.SetTryingForCoral(false); }),
      frc2::cmd::Sequence(
          frc2::cmd::WaitUntil([this] { return piv.IsClearOfFunnel().Get(); }),
          elev.GoToHeightCmd([] { return presets::elev::coral::l2; })),
      frc2::cmd::Sequence(
          frc2::cmd::WaitUntil([this] { return elev.GetHeight() > 30_in; }),
          piv.GoToAngleCmd([] { return presets::wrist::coral::l2; })));
}

frc2::CommandPtr Coordinator::GoToL2Algae() {
  return frc2::cmd::Parallel(
      frc2::cmd::RunOnce([this] { manip.SetTryingForCoral(false); }),
      manip.SuckUntilAlgae(),
      frc2::cmd::Sequence(
          frc2::cmd::WaitUntil([this] { return piv.IsClearOfFunnel().Get(); }),
          elev.GoToHeightCmd([] { return presets::elev::algae::l2; })),
      piv.GoToAngleCmd([] { return presets::wrist::algaeGrab; }));
}

frc2::CommandPtr Coordinator::GoToL2AlgaeAUTO() {
  return frc2::cmd::Parallel(
      frc2::cmd::RunOnce([this] { manip.SetTryingForCoral(false); }),
      frc2::cmd::Sequence(
          frc2::cmd::WaitUntil([this] { return piv.IsClearOfFunnel().Get(); }),
          elev.GoToHeightCmd([] { return presets::elev::algae::l2 - 8_in; })),
      piv.GoToAngleCmd([] { return presets::wrist::algaeGrab; }));
}

frc2::CommandPtr Coordinator::GoToL3Coral() {
  return frc2::cmd::Parallel(
      frc2::cmd::RunOnce([this] { manip.SetTryingForCoral(false); }),
      frc2::cmd::Sequence(
          frc2::cmd::WaitUntil([this] { return piv.IsClearOfFunnel().Get(); }),
          elev.GoToHeightCmd([] { return presets::elev::coral::l3; })),
      frc2::cmd::Sequence(
          frc2::cmd::WaitUntil([this] { return elev.GetHeight() > 30_in; }),
          piv.GoToAngleCmd([] { return presets::wrist::coral::l3; })));
}

frc2::CommandPtr Coordinator::GoToL3Algae() {
  return frc2::cmd::Parallel(
      frc2::cmd::RunOnce([this] { manip.SetTryingForCoral(false); }),
      manip.SuckUntilAlgae(),
      frc2::cmd::Sequence(
          frc2::cmd::WaitUntil([this] { return piv.IsClearOfFunnel().Get(); }),
          elev.GoToHeightCmd([] { return presets::elev::algae::l3; })),
      piv.GoToAngleCmd([] { return presets::wrist::algaeGrab; }));
}

frc2::CommandPtr Coordinator::GoToL4Coral() {
  return frc2::cmd::Parallel(
      frc2::cmd::RunOnce([this] { manip.SetTryingForCoral(false); }),
      frc2::cmd::Sequence(
          frc2::cmd::WaitUntil([this] { return piv.IsClearOfFunnel().Get(); }),
          elev.GoToHeightCmd([] { return presets::elev::coral::l4; })),
      frc2::cmd::Sequence(
          frc2::cmd::WaitUntil([this] { return elev.GetHeight() > 30_in; }),
          piv.GoToAngleCmd([] { return presets::wrist::coral::l4; })));
}

frc2::CommandPtr Coordinator::GoToNet() {
  return frc2::cmd::Sequence(
      piv.GoToAngleCmd([] { return presets::wrist::algaeHold; }),
      frc2::cmd::Parallel(
          frc2::cmd::RunOnce([this] { manip.SetTryingForCoral(false); }),
          frc2::cmd::Sequence(
              frc2::cmd::WaitUntil(
                  [this] { return piv.IsClearOfFunnel().Get(); }),
              elev.GoToHeightCmd([] { return presets::elev::algae::net; })),
          piv.GoToAngleCmd([] { return presets::wrist::algaeNet; })));
}
