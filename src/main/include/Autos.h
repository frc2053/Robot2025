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
        std::pair{TEST, pathplanner::PathPlannerAuto("Test").ToPtr()},
        std::pair{LEFT_EDGE_THREE_CORAL,
                  pathplanner::PathPlannerAuto("LeftEdgeThreeCoral").ToPtr()},
        std::pair{RIGHT_EDGE_THREE_CORAL,
                  pathplanner::PathPlannerAuto("RightEdgeThreeCoral").ToPtr()},
        std::pair{MIDDLE_CORAL_PROC,
                  pathplanner::PathPlannerAuto("MiddleCoralProc").ToPtr()});

    autoChooser.SetDefaultOption("Do Nothing", AutoSelector::NOTHING);
    autoChooser.AddOption("Test", AutoSelector::TEST);
    autoChooser.AddOption("Left Edge Three Coral",
                          AutoSelector::LEFT_EDGE_THREE_CORAL);
    autoChooser.AddOption("Right Edge Three Coral",
                          AutoSelector::RIGHT_EDGE_THREE_CORAL);
    autoChooser.AddOption("Middle Coral", AutoSelector::MIDDLE_CORAL_PROC);
    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
  }

  frc2::Command* GetSelectedCommand() { return selectCommand.get(); }

 private:
  void BindCommandsAndTriggers() {
    pathplanner::NamedCommands::registerCommand(
        "GetOutOfStarting", m_coordinator.GetOutOfStartingConfig());
    pathplanner::NamedCommands::registerCommand("Prime",
                                                m_coordinator.GoToCoralPrime());
    pathplanner::NamedCommands::registerCommand("L1", m_coordinator.GoToL1());
    pathplanner::NamedCommands::registerCommand("L2", m_coordinator.GoToL2());
    pathplanner::NamedCommands::registerCommand(
        "L2Algae", m_coordinator.GoToL2AlgaeAUTO());
    pathplanner::NamedCommands::registerCommand("L3", m_coordinator.GoToL3());
    pathplanner::NamedCommands::registerCommand("L4Coral",
                                                m_coordinator.GoToL4Coral());
    pathplanner::NamedCommands::registerCommand("L4", m_coordinator.GoToL4());
    pathplanner::NamedCommands::registerCommand(
        "Loading", m_coordinator.GoToLoading(true));
    pathplanner::NamedCommands::registerCommand(
        "Suck", frc2::cmd::RunOnce([this] { m_manipSub.Suck(); }));
    pathplanner::NamedCommands::registerCommand(
        "WaitForCoral", frc2::cmd::WaitUntil([this] {
          bool test = m_manipSub.GotCoralFR().Get();
          fmt::print("test: {}\n", test);
          return test;
        }));

    pathplanner::NamedCommands::registerCommand("SuckUntilAlgae",
                                                m_manipSub.SuckUntilAlgae());
    pathplanner::NamedCommands::registerCommand(
        "Score", m_manipSub.PoopPiece([] { return .5_s; }));
    pathplanner::NamedCommands::registerCommand(
        "ScoreAlgae", m_manipSub.PoopPiece([] { return 2_s; }));
    pathplanner::NamedCommands::registerCommand(
        "DriveToLeftReef",
        m_driveSub.AlignToReef([] { return true; }).WithTimeout(1.5_s));
    pathplanner::NamedCommands::registerCommand(
        "DriveToClosestAlgae", m_driveSub.AlignToAlgae().WithTimeout(1.5_s));
    pathplanner::NamedCommands::registerCommand(
        "DriveToRightReef",
        m_driveSub.AlignToReef([] { return false; }).WithTimeout(1.5_s));

    pathplanner::NamedCommands::registerCommand(
        "DesiredTagFL", m_driveSub.SetDesiredTag("Tag19"));
    pathplanner::NamedCommands::registerCommand(
        "DesiredTagBL", m_driveSub.SetDesiredTag("Tag20"));
    pathplanner::NamedCommands::registerCommand(
        "DesiredTagFR", m_driveSub.SetDesiredTag("Tag17"));
    pathplanner::NamedCommands::registerCommand(
        "DesiredTagBR", m_driveSub.SetDesiredTag("Tag22"));
  }

  enum AutoSelector {
    NOTHING,
    TEST,
    LEFT_EDGE_THREE_CORAL,
    RIGHT_EDGE_THREE_CORAL,
    MIDDLE_CORAL_PROC
  };

  frc::SendableChooser<AutoSelector> autoChooser;

  Drive& m_driveSub;
  Coordinator& m_coordinator;
  Manipulator& m_manipSub;

  frc2::CommandPtr selectCommand{frc2::cmd::None()};
};
