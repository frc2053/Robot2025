// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/controller/ElevatorFeedforward.h>
#include <frc2/command/SubsystemBase.h>

#include <memory>
#include <string>

#include <ctre/phoenix6/TalonFX.hpp>

#include "constants/ElevatorConstants.h"
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/SignalLogger.hpp"
#include "ctre/phoenix6/controls/CoastOut.hpp"
#include "ctre/phoenix6/controls/Follower.hpp"
#include "ctre/phoenix6/controls/MotionMagicExpoVoltage.hpp"
#include "ctre/phoenix6/sim/CANcoderSimState.hpp"
#include "frc/Alert.h"
#include "frc/controller/PIDController.h"
#include "frc/geometry/Pose3d.h"
#include "frc/simulation/ElevatorSim.h"
#include "frc/trajectory/ExponentialProfile.h"
#include "frc/trajectory/TrapezoidProfile.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/button/Trigger.h"
#include "frc2/command/sysid/SysIdRoutine.h"
#include "networktables/BooleanTopic.h"
#include "networktables/DoubleTopic.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/StructArrayTopic.h"
#include "str/GainTypes.h"
#include "str/SuperstructureDisplay.h"
#include "units/acceleration.h"
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
  void GoToHeight(units::meter_t newHeight);
  frc2::Trigger IsAtGoalHeight();
  void SetVoltage(units::volt_t volts);
  frc2::CommandPtr Coast();
  frc2::CommandPtr GoToHeightCmd(std::function<units::meter_t()> newHeight);
  frc2::CommandPtr SysIdElevatorQuasistaticVoltage(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdElevatorDynamicVoltage(frc2::sysid::Direction dir);
  frc2::CommandPtr TuneElevatorPID(std::function<bool()> isDone);
  void SetToZeroHeight();

 private:
  void ConfigureMotors();
  void ConfigureControlSignals();
  void UpdateNTEntries();
  units::meters_per_second_t GetElevatorVel();
  void SetElevatorGains(str::gains::linear::VoltLinearGainsHolder newGains,
                        units::volt_t kg, units::meters_per_second_t newMaxVel,
                        units::meters_per_second_squared_t newMaxAccel);
  units::meter_t ConvertEncPosToHeight(units::turn_t rots);
  units::turn_t ConvertHeightToEncPos(units::meter_t height);
  units::meters_per_second_t ConvertEncVelToHeightVel(
      units::turns_per_second_t radialVel);
  units::turns_per_second_t ConvertHeightVelToEncVel(
      units::meters_per_second_t vel);
  void LogElevatorVolts(frc::sysid::SysIdRoutineLog* log);
  ctre::phoenix6::hardware::TalonFX frontMotor{
      consts::elevator::can_ids::FRONT_MOTOR, "*"};
  ctre::phoenix6::hardware::TalonFX backMotor{
      consts::elevator::can_ids::BACK_MOTOR, "*"};

  units::meter_t currentHeight = 0_m;
  bool isAtGoalHeight = false;
  bool isCharacterizing = false;

  ctre::phoenix6::sim::TalonFXSimState& frontMotorSim =
      frontMotor.GetSimState();
  ctre::phoenix6::sim::TalonFXSimState& backMotorSim = backMotor.GetSimState();

  ctre::phoenix6::StatusSignal<units::turn_t> frontPositionSig =
      frontMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> frontVelocitySig =
      frontMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::volt_t> frontVoltageSig =
      frontMotor.GetMotorVoltage();

  ctre::phoenix6::StatusSignal<units::turn_t> backPositionSig =
      backMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> backVelocitySig =
      backMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::volt_t> backVoltageSig =
      backMotor.GetMotorVoltage();

  ctre::phoenix6::controls::VoltageOut elevatorVoltageSetter{0_V};
  ctre::phoenix6::controls::CoastOut coastSetter{};
  ctre::phoenix6::controls::Follower followerSetter{
      consts::elevator::can_ids::FRONT_MOTOR, false};

  str::gains::linear::VoltLinearGainsHolder currentGains{
      consts::elevator::gains::ELEVATOR_GAINS};
  units::volt_t currentKg{consts::elevator::gains::kG};

  frc::sim::ElevatorSim elevatorSim{consts::elevator::physical::MOTOR,
                                    consts::elevator::physical::GEARING,
                                    consts::elevator::physical::CARRIAGE_MASS,
                                    consts::elevator::physical::PULLEY_DIAM / 2,
                                    0_m,
                                    consts::elevator::physical::EXTENDED_HEIGHT,
                                    true,
                                    0_m,
                                    {0.00}};

  frc::ElevatorFeedforward ff{currentGains.kS, currentKg, currentGains.kV,
                              currentGains.kA};

  units::meters_per_second_t maxProfVel = 5_fps;
  units::meters_per_second_squared_t maxProfAccel = 10_fps_sq;

  frc::TrapezoidProfile<units::meter> trapProf{{maxProfVel, maxProfAccel}};
  frc::TrapezoidProfile<units::meter>::State trapSetpoint{};
  frc::TrapezoidProfile<units::meter>::State trapGoal{};

  frc::PIDController elevatorPid{currentGains.kP.value(),
                                 currentGains.kI.value(),
                                 currentGains.kD.value()};

  units::volt_t ffToSend{0_V};
  double pidOutput{0};

  frc2::sysid::SysIdRoutine elevatorSysIdVoltage{
      frc2::sysid::Config{std::nullopt, 10_V, std::nullopt, nullptr},
      frc2::sysid::Mechanism{
          [this](units::volt_t voltsToSend) { SetVoltage(voltsToSend); },
          [this](frc::sysid::SysIdRoutineLog* log) { LogElevatorVolts(log); },
          this, "elevator"}};

  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("Elevator")};
  nt::DoublePublisher currentHeightPub{
      nt->GetDoubleTopic("CurrentHeight").Publish()};
  nt::DoublePublisher currentVelPub{
      nt->GetDoubleTopic("CurrentVelocity").Publish()};
  nt::DoublePublisher heightPosSetpointPub{
      nt->GetDoubleTopic("HeightPosSetpoint").Publish()};
  nt::DoublePublisher heightVelSetpointPub{
      nt->GetDoubleTopic("HeightVelSetpoint").Publish()};
  nt::DoublePublisher heightGoalPub{nt->GetDoubleTopic("HeightGoal").Publish()};
  nt::BooleanPublisher isAtSetpointPub{
      nt->GetBooleanTopic("IsAtRequestedHeight").Publish()};
  str::SuperstructureDisplay& display;
  std::string frontAlertMsg{"Elevator Front Motor Config"};
  std::string backAlertMsg{"Elevator Back Motor Config"};
  frc::Alert configureFrontAlert{frontAlertMsg, frc::Alert::AlertType::kError};
  frc::Alert configureBackAlert{backAlertMsg, frc::Alert::AlertType::kError};
  std::string frontOptiAlertMsg{"Elevator Front Bus Optimization"};
  std::string backOptiAlertMsg{"Elevator Back Bus Optimization"};
  frc::Alert optiFrontAlert{frontOptiAlertMsg, frc::Alert::AlertType::kError};
  frc::Alert optiBackAlert{backOptiAlertMsg, frc::Alert::AlertType::kError};
  std::string signalFrequencyAlertStr{"Elevator Signal Frequency Set"};
  frc::Alert signalFrequencyAlert{signalFrequencyAlertStr,
                                  frc::Alert::AlertType::kError};
};
