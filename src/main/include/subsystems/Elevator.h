// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include "constants/ElevatorConstants.h"
#include "ctre/phoenix6/SignalLogger.hpp"
#include "frc/Alert.h"
#include "frc/simulation/ElevatorSim.h"
#include "frc2/command/sysid/SysIdRoutine.h"
#include "networktables/DoubleTopic.h"
#include "networktables/NetworkTableInstance.h"
#include "str/GainTypes.h"
#include "str/SuperstructureDisplay.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/current.h"
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
  void GoToHeight(units::meter_t newHeight);
  bool IsAtGoalHeight();
  void SetVoltage(units::volt_t volts);
  void SetTorqueCurrent(units::ampere_t amps);
  frc2::CommandPtr GoToHeight(std::function<units::meter_t()> newHeight);
  frc2::CommandPtr SysIdElevatorQuasistaticVoltage(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdElevatorDynamicVoltage(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdElevatorQuasistaticTorqueCurrent(
      frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdElevatorDynamicTorqueCurrent(
      frc2::sysid::Direction dir);

 private:
  void ConfigureMotors();
  void ConfigureControlSignals();
  void UpdateNTEntries();
  void SetElevatorGains(str::gains::radial::RadialGainsHolder newGains,
                        units::ampere_t kg);
  units::meter_t ConvertRadiansToHeight(units::radian_t rots);
  units::radian_t ConvertHeightToRadians(units::meter_t height);
  units::meters_per_second_t ConvertRadianVelToHeightVel(
      units::radians_per_second_t radialVel);
  units::radians_per_second_t ConvertHeightVelToRadianVel(
      units::meters_per_second_t vel);
  void LogElevatorVolts(frc::sysid::SysIdRoutineLog* log);
  void LogElevatorTorqueCurrent(frc::sysid::SysIdRoutineLog* log);
  ctre::phoenix6::hardware::TalonFX leftMotor{
      consts::elevator::can_ids::LEFT_MOTOR};
  ctre::phoenix6::hardware::TalonFX rightMotor{
      consts::elevator::can_ids::RIGHT_MOTOR};

  units::meter_t goalHeight = 0_m;
  units::meter_t currentHeight = 0_m;

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
  units::ampere_t currentKg{consts::elevator::gains::kG};

  frc::sim::ElevatorSim elevatorSim{consts::elevator::physical::MOTOR,
                                    consts::elevator::physical::GEARING,
                                    consts::elevator::physical::CARRIAGE_MASS,
                                    consts::elevator::physical::PULLEY_DIAM / 2,
                                    0_m,
                                    consts::elevator::physical::EXTENDED_HEIGHT,
                                    true,
                                    0_m,
                                    {0.00}};

  frc2::sysid::SysIdRoutine elevatorSysIdVoltage{
      frc2::sysid::Config{
          std::nullopt, 10_V, std::nullopt,
          [](frc::sysid::State state) {
            ctre::phoenix6::SignalLogger().WriteString(
                "SysIdElevator_State",
                frc::sysid::SysIdRoutineLog::StateEnumToString(state));
          }},
      frc2::sysid::Mechanism{
          [this](units::volt_t voltsToSend) { SetVoltage(voltsToSend); },
          [this](frc::sysid::SysIdRoutineLog* log) { LogElevatorVolts(log); },
          this, "elevator"}};

  frc2::sysid::SysIdRoutine elevatorSysIdTorqueCurrent{
      frc2::sysid::Config{
          (10_V / 1_s), 500_V, std::nullopt,
          [](frc::sysid::State state) {
            ctre::phoenix6::SignalLogger().WriteString(
                "SysIdElevator_State",
                frc::sysid::SysIdRoutineLog::StateEnumToString(state));
          }},
      frc2::sysid::Mechanism{[this](units::volt_t ampsToSend) {
                               SetTorqueCurrent(
                                   units::ampere_t{ampsToSend.value()});
                             },
                             [this](frc::sysid::SysIdRoutineLog* log) {
                               LogElevatorTorqueCurrent(log);
                             },
                             this, "elevator"}};

  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("Elevator")};
  nt::DoublePublisher currentHeightPub{
      nt->GetDoubleTopic("CurrentHeight").Publish()};
  nt::DoublePublisher heightSetpointPub{
      nt->GetDoubleTopic("HeightSetpoint").Publish()};

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
