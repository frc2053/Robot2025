// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/NetworkButton.h>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "Autos.h"
#include "frc/geometry/Pose2d.h"
#include "frc2/command/button/Trigger.h"
#include "str/SuperstructureDisplay.h"
#include "str/vision/VisionSystem.h"
#include "subsystems/AlgaeIntake.h"
#include "subsystems/Coordinator.h"
#include "subsystems/Drive.h"
#include "subsystems/Elevator.h"
#include "subsystems/Manipulator.h"
#include "subsystems/Pivot.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();
  Drive& GetDrive();
  Elevator& GetElevator();
  // Pivot& GetPivot();
  // Manipulator& GetManipulator();
  // Coordinator& GetCoordinator();
  // str::vision::VisionSystem& GetVision();
  // AlgaeIntake& GetAlgaeIntake();
  str::SuperstructureDisplay& GetSuperStructureDisplay();

 private:
  void ConfigureBindings();
  void ConfigureSysIdBinds();
  frc2::CommandPtr SteerVoltsSysIdCommands(std::function<bool()> fwd,
                                           std::function<bool()> quasistatic);
  frc2::CommandPtr SteerTorqueCurrentSysIdCommands(
      std::function<bool()> fwd, std::function<bool()> quasistatic);
  frc2::CommandPtr DriveSysIdCommands(std::function<bool()> fwd,
                                      std::function<bool()> quasistatic);
  frc2::CommandPtr WheelRadiusSysIdCommands(std::function<bool()> fwd);
  frc2::CommandPtr ElevatorVoltsSysIdCommands(
      std::function<bool()> fwd, std::function<bool()> quasistatic);
  frc2::CommandPtr PivotVoltsSysIdCommands(std::function<bool()> fwd,
                                           std::function<bool()> quasistatic);
  frc2::CommandPtr AlgaeIntakePivotVoltsSysIdCommands(
      std::function<bool()> fwd, std::function<bool()> quasistatic);
  frc2::CommandPtr HandleReturnToNeutralPosition();
  frc2::Trigger NoButtonsPressed();

  frc2::CommandXboxController driverJoystick{0};
  frc2::CommandXboxController operatorJoystick{1};

  str::SuperstructureDisplay display{};

  Drive driveSub{};
  Elevator elevatorSub{display};
  //   Pivot pivotSub{display};
  //   Manipulator manipSub{display};
  //   AlgaeIntake algaeintakeSub{display};
  //   Coordinator coordinator{elevatorSub, pivotSub, manipSub};

  // str::vision::VisionSystem vision;

  // Autos autos{driveSub, coordinator, manipSub};

  std::shared_ptr<nt::NetworkTable> tuningTable{
      nt::NetworkTableInstance::GetDefault().GetTable("Tuning")};
  frc2::NetworkButton steerTuneBtn{tuningTable, "SteerPidTuning"};
  frc2::NetworkButton driveTuneBtn{tuningTable, "DrivePidTuning"};
  frc2::NetworkButton steerSysIdVoltsBtn{tuningTable, "SteerSysIdVolts"};
  frc2::NetworkButton steerSysIdTorqueCurrentBtn{tuningTable,
                                                 "SteerSysIdTorqueCurrent"};
  frc2::NetworkButton driveSysIdBtn{tuningTable, "DriveSysId"};
  frc2::NetworkButton wheelRadiusBtn{tuningTable, "WheelRadius"};
  frc2::NetworkButton elevatorTuneBtn{tuningTable, "ElevatorPidTuning"};
  frc2::NetworkButton elevatorSysIdVoltsBtn{tuningTable, "ElevatorSysIdVolts"};
  frc2::NetworkButton pivotTuneBtn{tuningTable, "PivotPidTuning"};
  frc2::NetworkButton pivotSysIdVoltsBtn{tuningTable, "PivotSysIdVolts"};
  frc2::NetworkButton algaePivotTuneBtn{tuningTable, "AlgaePivotPidTuning"};
  frc2::NetworkButton algaePivotSysIdVoltsBtn{tuningTable,
                                              "AlgaePivotSysIdVolts"};
  frc2::NetworkButton coastElevatorBtn{tuningTable, "CoastElevator"};
  frc2::NetworkButton coastPivotBtn{tuningTable, "CoastPivot"};
};
