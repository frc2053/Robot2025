// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <numbers>

#include <ctre/phoenix6/TalonFX.hpp>

#include "constants/ArmConstants.h"
#include "frc/Alert.h"
#include "frc/simulation/SingleJointedArmSim.h"
#include "str/SuperstructureDisplay.h"
#include "units/voltage.h"

class Arm : public frc2::SubsystemBase {
 public:
  explicit Arm(str::SuperstructureDisplay& display);
  void OptimizeBusSignals();

  void Periodic() override;
  void SimulationPeriodic() override;
  units::radian_t GetAngle();
  void GoToAngle();
  void SetVoltage(units::volt_t volts);

 private:
  void ConfigureMotors();
  void ConfigureControlSignals();

  ctre::phoenix6::hardware::TalonFX pivotMotor{
      consts::arm::can_ids::PIVOT_MOTOR};

  ctre::phoenix6::sim::TalonFXSimState& pivotMotorSim =
      pivotMotor.GetSimState();

  ctre::phoenix6::StatusSignal<units::turn_t> pivotPositionSig =
      pivotMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> pivotVelocitySig =
      pivotMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::ampere_t> pivotTorqueCurrentSig =
      pivotMotor.GetTorqueCurrent();
  ctre::phoenix6::StatusSignal<units::volt_t> pivotVoltageSig =
      pivotMotor.GetMotorVoltage();

  ctre::phoenix6::controls::MotionMagicExpoTorqueCurrentFOC pivotAngleSetter{
      0_rad};
  ctre::phoenix6::controls::VoltageOut pivotVoltageSetter{0_V};
  ctre::phoenix6::controls::TorqueCurrentFOC pivotTorqueCurrentSetter{0_A};

  consts::arm::gains::holder currentGains{consts::arm::gains::ARM_GAINS};

  frc::sim::SingleJointedArmSim armSim{consts::arm::physical::MOTOR,
                                       consts::arm::physical::GEARING,
                                       consts::arm::physical::MOI,
                                       consts::arm::physical::ARM_LENGTH,
                                       consts::arm::physical::MIN_ANGLE,
                                       consts::arm::physical::MAX_ANGLE,
                                       true,
                                       0_rad,
                                       {0.005}};

  str::SuperstructureDisplay& display;
  std::string pivotAlertMsg{"Arm Pivot Motor Config"};
  frc::Alert configurePivotAlert{pivotAlertMsg, frc::Alert::AlertType::kError};
  std::string pivotOptiAlertMsg{"Pivot Bus Optimization"};
  frc::Alert optiPivotAlert{pivotOptiAlertMsg, frc::Alert::AlertType::kError};
  std::string signalFrequencyAlertStr{"Arm Signal Frequency Set"};
  frc::Alert signalFrequencyAlert{signalFrequencyAlertStr,
                                  frc::Alert::AlertType::kError};
};
