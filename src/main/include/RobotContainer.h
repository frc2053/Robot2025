// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/NetworkButton.h>

#include <functional>
#include <memory>

#include "Autos.h"
#include "str/SuperstructureDisplay.h"
#include "str/vision/VisionSystem.h"
#include "subsystems/Arm.h"
#include "subsystems/Drive.h"
#include "subsystems/Elevator.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();
  Drive& GetDrive();
  Elevator& GetElevator();
  Arm& GetArm();
  str::vision::VisionSystem& GetVision();
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

  frc2::CommandXboxController driverJoystick{0};

  str::SuperstructureDisplay display{};

  Drive driveSub{};
  Elevator elevatorSub{display};
  Arm armSub{display};
  str::vision::VisionSystem vision;

  Autos autos{driveSub};

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
};
