// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

// This method will be called once per scheduler run
void Elevator::Periodic() {
  display.SetElevatorHeight(0_in);
}
void Elevator::SimulationPeriodic() {}