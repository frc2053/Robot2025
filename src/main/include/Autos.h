// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include "subsystems/Coordinator.h"
#include "subsystems/Drive.h"
#include "subsystems/Manipulator.h"

class Autos {
 public:
  explicit Autos(Drive& driveSub, Coordinator& coordinator, Manipulator& manip)
      : m_driveSub{driveSub}, m_coordinator{coordinator}, m_manipSub{manip} {
    BindCommandsAndTriggers();

    selectCommand = frc2::cmd::Select<AutoSelector>(
        [this] { return autoChooser.GetSelected(); },
        std::pair{NOTHING, frc2::cmd::None()},
        std::pair{LEFT_EDGE_TWO_CORAL,
                  pathplanner::PathPlannerAuto("LeftEdgeTwoCoral").ToPtr()},
        std::pair{RIGHT_EDGE_TWO_CORAL,
                  pathplanner::PathPlannerAuto("RightEdgeTwoCoral").ToPtr()});

    autoChooser.SetDefaultOption("Do Nothing", AutoSelector::NOTHING);
    autoChooser.AddOption("Left Edge Two Coral",
                          AutoSelector::LEFT_EDGE_TWO_CORAL);
    autoChooser.AddOption("Right Edge Two Coral",
                          AutoSelector::RIGHT_EDGE_TWO_CORAL);

    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
  }

  frc2::Command* GetSelectedCommand() { return selectCommand.get(); }

 private:
  void BindCommandsAndTriggers() {
    pathplanner::NamedCommands::registerCommand(
        "GetOutOfStarting", m_coordinator.GetOutOfStartingConfig());
    pathplanner::NamedCommands::registerCommand("PrimeToScore",
                                                m_coordinator.GoToL2());
    pathplanner::NamedCommands::registerCommand("L4Coral",
                                                m_coordinator.GoToL4());
    pathplanner::NamedCommands::registerCommand("Loading",
                                                m_coordinator.GoToLoading());
    pathplanner::NamedCommands::registerCommand("WaitForCoral",
                                                m_manipSub.SuckUntilCoral());
    pathplanner::NamedCommands::registerCommand(
        "Score", m_manipSub.PoopPiece([] { return 1_s; }));
  }

  enum AutoSelector { NOTHING, LEFT_EDGE_TWO_CORAL, RIGHT_EDGE_TWO_CORAL };

  frc::SendableChooser<AutoSelector> autoChooser;

  Drive& m_driveSub;
  Coordinator& m_coordinator;
  Manipulator& m_manipSub;

  frc2::CommandPtr selectCommand{frc2::cmd::None()};
};
