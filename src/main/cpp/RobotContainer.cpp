// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>

#include "constants/SwerveConstants.h"
#include "frc2/command/sysid/SysIdRoutine.h"
#include "str/DriverstationUtils.h"
#include "str/vision/VisionSystem.h"
#include "subsystems/Drive.h"
#include "subsystems/Elevator.h"

RobotContainer::RobotContainer() {
  ConfigureBindings();
  ConfigureSysIdBinds();
}

void RobotContainer::ConfigureBindings() {
  driveSub.SetDefaultCommand(driveSub.DriveTeleop(
      [this] {
        return str::NegateIfRed(
            frc::ApplyDeadband<double>(-driverJoystick.GetLeftY(), .1) *
            consts::swerve::physical::PHY_CHAR.MaxLinearSpeed());
      },
      [this] {
        return str::NegateIfRed(
            frc::ApplyDeadband<double>(-driverJoystick.GetLeftX(), .1) *
            consts::swerve::physical::PHY_CHAR.MaxLinearSpeed());
      },
      [this] {
        return frc::ApplyDeadband<double>(-driverJoystick.GetRightX(), .1) *
               consts::swerve::physical::MAX_ROT_SPEED;
      }));

  driverJoystick.LeftBumper().OnTrue(manipSub.SuckUntilAlgae());
  driverJoystick.RightBumper().WhileTrue(manipSub.PoopPiece());

  driverJoystick.A().OnTrue(elevatorSub.GoToHeightCmd([] { return 5.5_ft; }));
  driverJoystick.B().OnTrue(elevatorSub.GoToHeightCmd([] { return 0_ft; }));

  driverJoystick.X().OnTrue(pivotSub.GoToAngleCmd([] { return 135_deg; }));
  driverJoystick.Y().OnTrue(pivotSub.GoToAngleCmd([] { return 0_deg; }));

  elevatorSub.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        elevatorSub.SetVoltage(
            frc::ApplyDeadband<double>(-driverJoystick.GetRightY(), .1) * 12_V);
      },
      {&elevatorSub}));

  pivotSub.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        pivotSub.SetVoltage(
            frc::ApplyDeadband<double>(-driverJoystick.GetRightX(), .1) * 12_V);
      },
      {&pivotSub}));
}

void RobotContainer::ConfigureSysIdBinds() {
  tuningTable->PutBoolean("SteerPidTuning", false);
  tuningTable->PutBoolean("DrivePidTuning", false);
  tuningTable->PutBoolean("SteerSysIdVolts", false);
  tuningTable->PutBoolean("SteerSysIdTorqueCurrent", false);
  tuningTable->PutBoolean("DriveSysId", false);
  tuningTable->PutBoolean("WheelRadius", false);
  tuningTable->PutBoolean("ElevatorPidTuning", false);
  tuningTable->PutBoolean("ElevatorSysIdVolts", false);
  tuningTable->PutBoolean("PivotPidTuning", false);
  tuningTable->PutBoolean("PivotSysIdVolts", false);
  tuningTable->PutBoolean("Quasistatic", true);
  tuningTable->PutBoolean("Forward", true);

  steerTuneBtn.OnTrue(
      driveSub.TuneSteerPID([this] { return !steerTuneBtn.Get(); }));
  driveTuneBtn.OnTrue(
      driveSub.TuneDrivePID([this] { return !driveTuneBtn.Get(); }));
  elevatorTuneBtn.OnTrue(
      elevatorSub.TuneElevatorPID([this] { return !elevatorTuneBtn.Get(); }));
  pivotTuneBtn.OnTrue(
      pivotSub.TunePivotPID([this] { return !pivotTuneBtn.Get(); }));

  steerSysIdVoltsBtn.WhileTrue(SteerVoltsSysIdCommands(
      [this] { return tuningTable->GetBoolean("Forward", true); },
      [this] { return tuningTable->GetBoolean("Quasistatic", true); }));

  steerSysIdTorqueCurrentBtn.WhileTrue(SteerTorqueCurrentSysIdCommands(
      [this] { return tuningTable->GetBoolean("Forward", true); },
      [this] { return tuningTable->GetBoolean("Quasistatic", true); }));

  driveSysIdBtn.WhileTrue(DriveSysIdCommands(
      [this] { return tuningTable->GetBoolean("Forward", true); },
      [this] { return tuningTable->GetBoolean("Quasistatic", true); }));

  wheelRadiusBtn.WhileTrue(WheelRadiusSysIdCommands(
      [this] { return tuningTable->GetBoolean("Forward", true); }));

  elevatorSysIdVoltsBtn.WhileTrue(ElevatorVoltsSysIdCommands(
      [this] { return tuningTable->GetBoolean("Forward", true); },
      [this] { return tuningTable->GetBoolean("Quasistatic", true); }));

  pivotSysIdVoltsBtn.WhileTrue(PivotVoltsSysIdCommands(
      [this] { return tuningTable->GetBoolean("Forward", true); },
      [this] { return tuningTable->GetBoolean("Quasistatic", true); }));
}

frc2::CommandPtr RobotContainer::SteerVoltsSysIdCommands(
    std::function<bool()> fwd, std::function<bool()> quasistatic) {
  return frc2::cmd::Either(
      frc2::cmd::Either(
          driveSub.SysIdSteerQuasistaticVoltage(
              frc2::sysid::Direction::kForward),
          driveSub.SysIdSteerDynamicVoltage(frc2::sysid::Direction::kForward),
          quasistatic),
      frc2::cmd::Either(
          driveSub.SysIdSteerQuasistaticVoltage(
              frc2::sysid::Direction::kReverse),
          driveSub.SysIdSteerDynamicVoltage(frc2::sysid::Direction::kReverse),
          quasistatic),
      fwd);
}

frc2::CommandPtr RobotContainer::SteerTorqueCurrentSysIdCommands(
    std::function<bool()> fwd, std::function<bool()> quasistatic) {
  return frc2::cmd::Either(
      frc2::cmd::Either(driveSub.SysIdSteerQuasistaticTorqueCurrent(
                            frc2::sysid::Direction::kForward),
                        driveSub.SysIdSteerDynamicTorqueCurrent(
                            frc2::sysid::Direction::kForward),
                        quasistatic),
      frc2::cmd::Either(driveSub.SysIdSteerQuasistaticTorqueCurrent(
                            frc2::sysid::Direction::kReverse),
                        driveSub.SysIdSteerDynamicTorqueCurrent(
                            frc2::sysid::Direction::kReverse),
                        quasistatic),
      fwd);
}

frc2::CommandPtr RobotContainer::ElevatorVoltsSysIdCommands(
    std::function<bool()> fwd, std::function<bool()> quasistatic) {
  return frc2::cmd::Either(
      frc2::cmd::Either(elevatorSub.SysIdElevatorQuasistaticVoltage(
                            frc2::sysid::Direction::kForward),
                        elevatorSub.SysIdElevatorDynamicVoltage(
                            frc2::sysid::Direction::kForward),
                        quasistatic),
      frc2::cmd::Either(elevatorSub.SysIdElevatorQuasistaticVoltage(
                            frc2::sysid::Direction::kReverse),
                        elevatorSub.SysIdElevatorDynamicVoltage(
                            frc2::sysid::Direction::kReverse),
                        quasistatic),
      fwd);
}

frc2::CommandPtr RobotContainer::PivotVoltsSysIdCommands(
    std::function<bool()> fwd, std::function<bool()> quasistatic) {
  return frc2::cmd::Either(
      frc2::cmd::Either(
          pivotSub.SysIdPivotQuasistaticVoltage(
              frc2::sysid::Direction::kForward),
          pivotSub.SysIdPivotDynamicVoltage(frc2::sysid::Direction::kForward),
          quasistatic),
      frc2::cmd::Either(
          pivotSub.SysIdPivotQuasistaticVoltage(
              frc2::sysid::Direction::kReverse),
          pivotSub.SysIdPivotDynamicVoltage(frc2::sysid::Direction::kReverse),
          quasistatic),
      fwd);
}

frc2::CommandPtr RobotContainer::DriveSysIdCommands(
    std::function<bool()> fwd, std::function<bool()> quasistatic) {
  return frc2::cmd::Either(
      frc2::cmd::Either(driveSub.SysIdDriveQuasistaticTorqueCurrent(
                            frc2::sysid::Direction::kForward),
                        driveSub.SysIdDriveDynamicTorqueCurrent(
                            frc2::sysid::Direction::kForward),
                        quasistatic),
      frc2::cmd::Either(driveSub.SysIdDriveQuasistaticTorqueCurrent(
                            frc2::sysid::Direction::kReverse),
                        driveSub.SysIdDriveDynamicTorqueCurrent(
                            frc2::sysid::Direction::kReverse),
                        quasistatic),
      fwd);
}

frc2::CommandPtr RobotContainer::WheelRadiusSysIdCommands(
    std::function<bool()> fwd) {
  return frc2::cmd::Either(
      driveSub.WheelRadius(frc2::sysid::Direction::kForward),
      driveSub.WheelRadius(frc2::sysid::Direction::kReverse), fwd);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return autos.GetSelectedCommand();
}

Drive& RobotContainer::GetDrive() {
  return driveSub;
}

Elevator& RobotContainer::GetElevator() {
  return elevatorSub;
}

Pivot& RobotContainer::GetPivot() {
  return pivotSub;
}

Manipulator& RobotContainer::GetManipulator() {
  return manipSub;
}

str::vision::VisionSystem& RobotContainer::GetVision() {
  return vision;
}

str::SuperstructureDisplay& RobotContainer::GetSuperStructureDisplay() {
  return display;
}
