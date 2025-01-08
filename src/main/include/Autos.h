// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include "subsystems/Drive.h"

class Autos {
 public:
  explicit Autos(Drive& driveSub) : m_driveSub{driveSub} {
    BindCommandsAndTriggers();

    selectCommand = frc2::cmd::Select<AutoSelector>(
        [this] { return autoChooser.GetSelected(); },
        std::pair{NOTHING, frc2::cmd::None()},
        std::pair{TEST, pathplanner::PathPlannerAuto("Test").ToPtr()},
        std::pair{TESTPP, pathplanner::PathPlannerAuto("TestPP").ToPtr()});

    autoChooser.SetDefaultOption("Do Nothing", AutoSelector::TEST);
    autoChooser.AddOption("Test", AutoSelector::TEST);
    autoChooser.AddOption("TestPP", AutoSelector::TESTPP);

    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
  }

  frc2::Command* GetSelectedCommand() { return selectCommand.get(); }

 private:
  void BindCommandsAndTriggers() {
    pathplanner::NamedCommands::registerCommand(
        "Test", frc2::cmd::Print("Test named command"));
  }

  enum AutoSelector { NOTHING, TEST, TESTPP };

  frc::SendableChooser<AutoSelector> autoChooser;

  Drive& m_driveSub;

  frc2::CommandPtr selectCommand{frc2::cmd::None()};
};
