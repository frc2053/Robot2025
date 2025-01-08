// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "Robot.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/Threads.h>
#include <frc2/command/CommandScheduler.h>

#include <ctre/phoenix6/SignalLogger.hpp>

#include "constants/SwerveConstants.h"
#include <wpinet/WebServer.h>
#include <frc/Filesystem.h>

Robot::Robot() {
  // DANGEROUS MAKE SURE CODE DOESN'T BLOCK!!!
  frc::SetCurrentThreadPriority(true, 15);
  ctre::phoenix6::SignalLogger::EnableAutoLogging(true);
  ctre::phoenix6::SignalLogger::Start();
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  AddPeriodic([this] { m_container.GetDrive().UpdateOdom(); },
              1 / consts::swerve::ODOM_UPDATE_RATE, 2_ms);
  wpi::WebServer::GetInstance().Start(5800,
                                      frc::filesystem::GetDeployDirectory());
}

void Robot::RobotPeriodic() {
  units::second_t now = frc::Timer::GetFPGATimestamp();
  units::second_t loopTime = now - lastTotalLoopTime;
  loopTimePub.Set((1 / loopTime).value());

  frc2::CommandScheduler::GetInstance().Run();
  // UpdateVision();

  lastTotalLoopTime = now;
  matchTimePub.Set(frc::DriverStation::GetMatchTime().value());
  battVoltagePub.Set(frc::RobotController::GetBatteryVoltage().value());

  elevatorArm.Periodic();
}

void Robot::SimulationPeriodic() {
  // m_container.GetVision().SimulationPeriodic(
  //     m_container.GetDrive().GetOdomPose());
}

void Robot::UpdateVision() {
  // auto visionEstimates = m_container.GetVision().GetCameraEstimatedPoses(
  //     frc::Pose3d{m_container.GetDrive().GetRobotPose()});
  // auto stdDevs = m_container.GetVision().GetPoseStdDevs(visionEstimates);

  // frc::Pose3d pose = frc::Pose3d{m_container.GetDrive().GetRobotPose()};

  // m_container.GetVision().UpdateCameraPositionVis(pose);

  // int i = 0;
  // for (const auto& est : visionEstimates) {
  //   if (est.has_value()) {
  //     m_container.GetDrive().AddVisionMeasurement(
  //         est.value().estimatedPose.ToPose2d(), est.value().timestamp,
  //         stdDevs[i].value());
  //   }
  //   i++;
  // }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }

  elevatorArm.CalculateTraj();
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
