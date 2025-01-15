// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include "constants/ElevatorConstants.h"
#include "frc/Alert.h"
#include "frc/simulation/ElevatorSim.h"
#include "str/GainTypes.h"
#include "str/SuperstructureDisplay.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/velocity.h"
#include "units/voltage.h"

class Elevator : public frc2::SubsystemBase {
 public:
  explicit Elevator(str::SuperstructureDisplay& display);
  void OptimizeBusSignals();

  void Periodic() override;
  void SimulationPeriodic() override;
  units::meter_t GetHeight();
  void GoToHeight();
  void SetVoltage(units::volt_t volts);

 private:
  void ConfigureMotors();
  void ConfigureControlSignals();
  units::meter_t ConvertRadiansToHeight(units::radian_t rots);
  units::radian_t ConvertHeightToRadians(units::meter_t height);
  units::meters_per_second_t ConvertRadianVelToHeightVel(
      units::radians_per_second_t radialVel);
  units::radians_per_second_t ConvertHeightVelToRadianVel(
      units::meters_per_second_t vel);

  ctre::phoenix6::hardware::TalonFX leftMotor{
      consts::elevator::can_ids::LEFT_MOTOR};
  ctre::phoenix6::hardware::TalonFX rightMotor{
      consts::elevator::can_ids::RIGHT_MOTOR};

  ctre::phoenix6::sim::TalonFXSimState& leftMotorSim = leftMotor.GetSimState();
  ctre::phoenix6::sim::TalonFXSimState& rightMotorSim =
      rightMotor.GetSimState();

  ctre::phoenix6::StatusSignal<units::turn_t> leftPositionSig =
      leftMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> leftVelocitySig =
      leftMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::ampere_t> leftTorqueCurrentSig =
      leftMotor.GetTorqueCurrent();
  ctre::phoenix6::StatusSignal<units::volt_t> leftVoltageSig =
      leftMotor.GetMotorVoltage();

  ctre::phoenix6::StatusSignal<units::turn_t> rightPositionSig =
      rightMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> rightVelocitySig =
      rightMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::ampere_t> rightTorqueCurrentSig =
      rightMotor.GetTorqueCurrent();
  ctre::phoenix6::StatusSignal<units::volt_t> rightVoltageSig =
      rightMotor.GetMotorVoltage();

  ctre::phoenix6::controls::MotionMagicExpoTorqueCurrentFOC
      elevatorHeightSetter{0_rad};
  ctre::phoenix6::controls::VoltageOut elevatorVoltageSetter{0_V};
  ctre::phoenix6::controls::TorqueCurrentFOC elevatorTorqueCurrentSetter{0_A};

  str::gains::radial::RadialGainsHolder currentGains{
      consts::elevator::gains::ELEVATOR_GAINS};

  frc::sim::ElevatorSim elevatorSim{consts::elevator::physical::MOTOR,
                                    consts::elevator::physical::GEARING,
                                    consts::elevator::physical::CARRIAGE_MASS,
                                    consts::elevator::physical::PULLEY_DIAM / 2,
                                    0_m,
                                    consts::elevator::physical::EXTENDED_HEIGHT,
                                    true,
                                    0_m,
                                    {0.005}};

  str::SuperstructureDisplay& display;
  std::string leftAlertMsg{"Elevator Left Motor Config"};
  std::string rightAlertMsg{"Elevator Right Motor Config"};
  frc::Alert configureLeftAlert{leftAlertMsg, frc::Alert::AlertType::kError};
  frc::Alert configureRightAlert{rightAlertMsg, frc::Alert::AlertType::kError};
  std::string leftOptiAlertMsg{"Elevator Left Bus Optimization"};
  std::string rightOptiAlertMsg{"Elevator Right Bus Optimization"};
  frc::Alert optiLeftAlert{leftOptiAlertMsg, frc::Alert::AlertType::kError};
  frc::Alert optiRightAlert{rightOptiAlertMsg, frc::Alert::AlertType::kError};
  std::string signalFrequencyAlertStr{"Elevator Signal Frequency Set"};
  frc::Alert signalFrequencyAlert{signalFrequencyAlertStr,
                                  frc::Alert::AlertType::kError};
};
