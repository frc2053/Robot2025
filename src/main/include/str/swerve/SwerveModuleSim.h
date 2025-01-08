// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/simulation/DCMotorSim.h>

#include <ctre/phoenix6/sim/CANcoderSimState.hpp>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>

#include "str/swerve/SwerveModuleHelpers.h"

namespace str::swerve {

class SwerveModuleSim {
 public:
  explicit SwerveModuleSim(
      ModuleConstants constants, ModulePhysicalCharacteristics physicalAttrib,
      ctre::phoenix6::sim::TalonFXSimState& driveSimState,
      ctre::phoenix6::sim::TalonFXSimState& steerSimState,
      ctre::phoenix6::sim::CANcoderSimState& steerEncoderSimState);

  frc::SwerveModuleState Update(units::volt_t supplyVoltage);
  units::ampere_t GetSteerCurrentDraw() const;
  units::ampere_t GetDriveCurrentDraw() const;

 private:
  units::volt_t AddFrictionVoltage(units::volt_t outputVoltage,
                                   units::volt_t frictionVoltage);

  frc::sim::DCMotorSim driveSim;
  frc::sim::DCMotorSim steerSim;

  ctre::phoenix6::sim::TalonFXSimState& driveSimState;
  ctre::phoenix6::sim::TalonFXSimState& steerSimState;
  ctre::phoenix6::sim::CANcoderSimState& steerEncoderSimState;

  const bool driveInverted;
  const bool steerInverted;

  units::volt_t driveFrictionVoltage;
  units::volt_t steerFrictionVoltage;

  const units::scalar_t driveGearing;
  const units::scalar_t steerGearing;

  const units::meter_t wheelRadius;
};

}  // namespace str::swerve
