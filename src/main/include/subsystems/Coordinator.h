// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include "frc2/command/CommandPtr.h"
#include "subsystems/Elevator.h"
#include "subsystems/L1.h"
#include "subsystems/Manipulator.h"
#include "subsystems/Pivot.h"

class Coordinator {
 public:
  Coordinator(Elevator& elevator, Pivot& pivot, Manipulator& manipulator,
              L1& l1Manip);

  frc2::CommandPtr GoToL1();
  frc2::CommandPtr GoToL2();
  frc2::CommandPtr GoToL3();
  frc2::CommandPtr GoToL4();
  frc2::CommandPtr GoToAlgaeHold();
  frc2::CommandPtr GoToLoading(bool override = false);
  frc2::CommandPtr GetOutOfStartingConfig();
  frc2::CommandPtr PrimeCoral(std::function<units::radian_t()> finalAngle);
  frc2::CommandPtr WaitForPriming();
  frc2::CommandPtr GoToCoralPrime();
  frc2::CommandPtr GoToL4Coral();
  frc2::CommandPtr GoToL2AlgaeAUTO();
  frc2::CommandPtr Climb();

 private:
  frc2::CommandPtr GoToL1Coral();
  frc2::CommandPtr GoToAlgaeProcess();
  frc2::CommandPtr GoToL2Coral();
  frc2::CommandPtr GoToL2Algae();
  frc2::CommandPtr GoToL3Coral();
  frc2::CommandPtr GoToL3Algae();
  frc2::CommandPtr GoToNet();

  Elevator& elev;
  Pivot& piv;
  Manipulator& manip;
  L1& l1;
};
