// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include "constants/ElevatorConstants.h"
#include "frc/simulation/ElevatorSim.h"
#include "frc/system/plant/DCMotor.h"
#include "str/SuperstructureDisplay.h"

class Elevator : public frc2::SubsystemBase {
 public:
  explicit Elevator(str::SuperstructureDisplay& display) : display{display} {}
  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  ctre::phoenix6::hardware::TalonFX leftMotor{
      consts::elevator::can_ids::LEFT_MOTOR};
  ctre::phoenix6::hardware::TalonFX rightMotor{
      consts::elevator::can_ids::LEFT_MOTOR};

  ctre::phoenix6::sim::TalonFXSimState& leftMotorSim = leftMotor.GetSimState();
  ctre::phoenix6::sim::TalonFXSimState& rightMotorSim =
      rightMotor.GetSimState();

  frc::DCMotor elevatorGearbox{frc::DCMotor::Falcon500FOC(2)};

  frc::sim::ElevatorSim elevatorSim{elevatorGearbox,
                                    consts::elevator::physical::GEARING,
                                    consts::elevator::physical::CARRIAGE_MASS,
                                    consts::elevator::physical::PULLEY_DIAM / 2,
                                    0_m,
                                    consts::elevator::physical::EXTENDED_HEIGHT,
                                    true,
                                    0_m,
                                    {0.005}};

  str::SuperstructureDisplay& display;
};
