// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <str/swerve/SwerveDrive.h>

#include <optional>

#include "RobotContainer.h"
#include "str/armtraj/armtraj.h"

class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotPeriodic() override;
  void SimulationPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;
  void TestInit() override;
  void TestPeriodic() override;
  void TestExit() override;

 private:
  void UpdateVision();

  frc2::Command* m_autonomousCommand = nullptr;

  RobotContainer m_container;
  str::ArmTraj elevatorArm;

  units::second_t lastTotalLoopTime;
  nt::DoublePublisher loopTimePub{nt::NetworkTableInstance::GetDefault()
                                      .GetTable("Metadata")
                                      ->GetDoubleTopic("RobotPeriodicLoopRate")
                                      .Publish()};
  nt::DoublePublisher matchTimePub{nt::NetworkTableInstance::GetDefault()
                                       .GetTable("Metadata")
                                       ->GetDoubleTopic("MatchTime")
                                       .Publish()};
  nt::DoublePublisher battVoltagePub{nt::NetworkTableInstance::GetDefault()
                                         .GetTable("Metadata")
                                         ->GetDoubleTopic("BatteryVoltage")
                                         .Publish()};
};
