// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/Coordinator.h"

#include "constants/Presets.h"
#include "frc2/command/Commands.h"
#include "subsystems/Manipulator.h"

Coordinator::Coordinator(Elevator& elevator, Pivot& pivot,
                         Manipulator& manipulator)
    : elev{elevator}, piv{pivot}, manip{manipulator} {}

frc2::CommandPtr Coordinator::GoToL1() {
  return frc2::cmd::Either(GoToL1Coral(), GoToAlgaeProcess(),
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

frc2::CommandPtr Coordinator::GoToLoading() {
  return frc2::cmd::Parallel(
      elev.GoToHeightCmd([] { return presets::elev::coral::loading; }),
      piv.GoToAngleCmd([] { return presets::wrist::coral::loading; }));
}

frc2::CommandPtr Coordinator::GoToL1Coral() {
  return frc2::cmd::Parallel(
      elev.GoToHeightCmd([] { return presets::elev::coral::l1; }),
      piv.GoToAngleCmd([] { return presets::wrist::coral::l1; }));
}

frc2::CommandPtr Coordinator::GoToAlgaeProcess() {
  return frc2::cmd::Parallel(
      elev.GoToHeightCmd([] { return presets::elev::algae::processor; }),
      piv.GoToAngleCmd([] { return presets::wrist::algaeProcess; }));
}

frc2::CommandPtr Coordinator::GoToL2Coral() {
  return frc2::cmd::Parallel(
      piv.GoToAngleCmd([] { return presets::wrist::coral::l2; }),
      frc2::cmd::Sequence(
          frc2::cmd::WaitUntil([this] { return piv.IsClearOfFunnel().Get(); }),
          elev.GoToHeightCmd([] { return presets::elev::coral::l2; })));
}

frc2::CommandPtr Coordinator::GoToL2Algae() {
  return frc2::cmd::Parallel(
      elev.GoToHeightCmd([] { return presets::elev::algae::l2; }),
      piv.GoToAngleCmd([] { return presets::wrist::algaeGrab; }));
}

frc2::CommandPtr Coordinator::GoToL3Coral() {
  return frc2::cmd::Parallel(
      piv.GoToAngleCmd([] { return presets::wrist::coral::l3; }),
      frc2::cmd::Sequence(
          frc2::cmd::WaitUntil([this] { return piv.IsClearOfFunnel().Get(); }),
          elev.GoToHeightCmd([] { return presets::elev::coral::l3; })));
}

frc2::CommandPtr Coordinator::GoToL3Algae() {
  return frc2::cmd::Parallel(
      elev.GoToHeightCmd([] { return presets::elev::algae::l3; }),
      piv.GoToAngleCmd([] { return presets::wrist::algaeGrab; }));
}

frc2::CommandPtr Coordinator::GoToL4Coral() {
  return frc2::cmd::Parallel(
      piv.GoToAngleCmd([] { return presets::wrist::coral::l4; }),
      frc2::cmd::Sequence(
          frc2::cmd::WaitUntil([this] { return piv.IsClearOfFunnel().Get(); }),
          elev.GoToHeightCmd([] { return presets::elev::coral::l4; })));
}

frc2::CommandPtr Coordinator::GoToNet() {
  return frc2::cmd::Parallel(
      elev.GoToHeightCmd([] { return presets::elev::algae::net; }),
      piv.GoToAngleCmd([] { return presets::wrist::algaeNet; }));
}
