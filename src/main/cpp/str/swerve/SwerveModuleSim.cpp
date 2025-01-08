// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/swerve/SwerveModuleSim.h"

#include <frc/system/plant/LinearSystemId.h>

#include "str/swerve/SwerveModuleHelpers.h"

using namespace str::swerve;

SwerveModuleSim::SwerveModuleSim(
    ModuleConstants constants, ModulePhysicalCharacteristics physicalAttrib,
    ctre::phoenix6::sim::TalonFXSimState& driveSimState,
    ctre::phoenix6::sim::TalonFXSimState& steerSimState,
    ctre::phoenix6::sim::CANcoderSimState& steerEncoderSimState)
    : driveSim(frc::LinearSystemId::DCMotorSystem(physicalAttrib.driveMotor,
                                                  0.01_kg_sq_m,
                                                  physicalAttrib.driveGearing),
               physicalAttrib.driveMotor),
      steerSim(frc::LinearSystemId::DCMotorSystem(physicalAttrib.steerMotor,
                                                  0.01_kg_sq_m,
                                                  physicalAttrib.steerGearing),
               physicalAttrib.steerMotor),
      driveSimState(driveSimState),
      steerSimState(steerSimState),
      steerEncoderSimState(steerEncoderSimState),
      driveInverted(constants.invertDrive),
      steerInverted(constants.invertSteer),
      driveFrictionVoltage(physicalAttrib.driveFrictionVoltage),
      steerFrictionVoltage(physicalAttrib.steerFrictionVoltage),
      driveGearing(physicalAttrib.driveGearing),
      steerGearing(physicalAttrib.steerGearing),
      wheelRadius(physicalAttrib.wheelRadius) {
  driveSimState.SetRawRotorPosition(0_rad);
  driveSimState.SetRotorVelocity(0_rad_per_s);
  steerSimState.SetRawRotorPosition(0_rad);
  steerSimState.SetRotorVelocity(0_rad_per_s);
  steerEncoderSimState.SetRawPosition(0_rad);
  steerEncoderSimState.SetVelocity(0_rad_per_s);
  driveSim.SetState(0_rad, 0_rad_per_s);
  steerSim.SetState(0_rad, 0_rad_per_s);
}

frc::SwerveModuleState SwerveModuleSim::Update(units::volt_t supplyVoltage) {
  driveSimState.Orientation =
      ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
  steerSimState.Orientation =
      ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;

  driveSimState.SetSupplyVoltage(supplyVoltage);
  steerSimState.SetSupplyVoltage(supplyVoltage);
  steerEncoderSimState.SetSupplyVoltage(supplyVoltage);

  driveSim.SetInputVoltage(AddFrictionVoltage(driveSimState.GetMotorVoltage(),
                                              driveFrictionVoltage));
  steerSim.SetInputVoltage(AddFrictionVoltage(steerSimState.GetMotorVoltage(),
                                              steerFrictionVoltage));

  driveSim.Update(1 / 50_Hz);
  steerSim.Update(1 / 50_Hz);

  driveSimState.SetRawRotorPosition(driveSim.GetAngularPosition() *
                                    driveGearing);
  driveSimState.SetRotorVelocity(driveSim.GetAngularVelocity() * driveGearing);

  steerSimState.SetRawRotorPosition(steerSim.GetAngularPosition() *
                                    steerGearing);
  steerSimState.SetRotorVelocity(steerSim.GetAngularVelocity() * steerGearing);

  steerEncoderSimState.SetRawPosition(steerSim.GetAngularPosition());
  steerEncoderSimState.SetVelocity(steerSim.GetAngularVelocity());

  return frc::SwerveModuleState{
      (driveSim.GetAngularVelocity() / 1_rad) * wheelRadius,
      frc::Rotation2d{steerSim.GetAngularPosition()}};
}

units::volt_t SwerveModuleSim::AddFrictionVoltage(
    units::volt_t outputVoltage, units::volt_t frictionVoltage) {
  if (units::math::abs(outputVoltage) < frictionVoltage) {
    outputVoltage = 0_V;
  } else if (outputVoltage > 0_V) {
    outputVoltage -= frictionVoltage;
  } else {
    outputVoltage += frictionVoltage;
  }
  return outputVoltage;
}

units::ampere_t SwerveModuleSim::GetSteerCurrentDraw() const {
  return steerSim.GetCurrentDraw();
}

units::ampere_t SwerveModuleSim::GetDriveCurrentDraw() const {
  return driveSim.GetCurrentDraw();
}
