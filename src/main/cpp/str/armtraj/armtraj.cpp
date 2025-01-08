#include <chrono>
#include <iostream>
#include <numbers>
#include "units/angle.h"
#include "str/armtraj/armtraj.h"

using namespace str;

int ArmTraj::CalculateTraj() {
  using namespace std::chrono_literals;

  // ELEVATOR INIT CONDITIONS
  constexpr double kElevatorStartHeight = 0;         // m
  constexpr double kElevatorEndHeight = 1.63449;     // m
  constexpr double kElevatorMaxVelocity = 1.962912;  // m/s
  constexpr double kElevatorMaxAcceleration = 2.0;   // m/s²

  // ARM INIT CONDITIONS
  constexpr double kArmStartAngle = 0.0;                          // rad
  constexpr double kArmEndAngle = std::numbers::pi;               // rad
  constexpr double kArmMaxVelocity = 2.0 * std::numbers::pi;      // rad/s
  constexpr double kArmMaxAcceleration = 4.0 * std::numbers::pi;  // rad/s²

  // Elevator parameters
  constexpr double kElevatorMotorKt = 0.1;   // N⋅m/A (torque constant)
  constexpr double kElevatorMotorR = 1.0;    // Ω (armature resistance)
  constexpr double kElevatorMotorL = 0.001;  // H (armature inductance)
  constexpr double kElevatorMotorJ = 0.01;   // kg⋅m² (rotor inertia)
  constexpr double kElevatorMotorB = 0.1;    // N⋅m⋅s/rad (viscous friction)
  constexpr double kElevatorMaxCurrent = 40.0;  // A
  constexpr double kElevatorMaxVoltage = 12.0;  // V
  constexpr double kElevatorGearRatio = 6.0;    // Output/Input ratio
  constexpr double kElevatorMass = 9.0;  // kg (mass of elevator carriage)
  constexpr double kEndEffectorMaxHeight = 1.8;  // m

  // Arm parameters
  constexpr double kArmMotorKt = 0.08;     // N⋅m/A (torque constant)
  constexpr double kArmMotorR = 1.0;       // Ω (armature resistance)
  constexpr double kArmMotorL = 0.001;     // H (armature inductance)
  constexpr double kArmMotorJ = 0.005;     // kg⋅m² (rotor inertia)
  constexpr double kArmMotorB = 0.05;      // N⋅m⋅s/rad (viscous friction)
  constexpr double kArmMaxCurrent = 40.0;  // A
  constexpr double kArmMaxVoltage = 12.0;  // V
  constexpr double kArmGearRatio = 20.0;   // Output/Input ratio
  constexpr double kArmMass = 1.0;         // kg (mass of arm)

  constexpr double kGravity = 9.81;  // m/s²

  constexpr std::chrono::duration<double> kTotalTime = 4s;
  constexpr auto dt = kTotalTime / N;

  // Motor variables for both motors
  auto elevatorCurrent =
      problem.DecisionVariable(1, N);  // Elevator motor current
  auto elevatorVoltage =
      problem.DecisionVariable(1, N);                // Elevator motor voltage
  auto armCurrent = problem.DecisionVariable(1, N);  // Arm motor current
  auto armVoltage = problem.DecisionVariable(1, N);  // Arm motor voltage

  for (int k = 0; k < N; ++k) {
    // Elevator dynamics with motor
    // Convert linear motion to rotary for the motor
    // Assuming a rack and pinion or pulley system with radius r = 1/gearRatio
    // Linear velocity = angular velocity * radius
    // Therefore: angular velocity = linear velocity * gearRatio

    // Elevator torque balance including gravity
    problem.SubjectTo(
        kElevatorMotorKt * elevatorCurrent(0, k) * kElevatorGearRatio ==
        kElevatorMotorJ * elevatorAccel(0, k) * kElevatorGearRatio *
                kElevatorGearRatio +
            kElevatorMotorB * elevator(1, k) * kElevatorGearRatio +
            kElevatorMass * kGravity / kElevatorGearRatio);

    // Elevator voltage equation
    if (k < N - 1) {
      problem.SubjectTo(
          elevatorVoltage(0, k) ==
          elevatorCurrent(0, k) * kElevatorMotorR +
              kElevatorMotorL *
                  (elevatorCurrent(0, k + 1) - elevatorCurrent(0, k)) /
                  dt.count() +
              kElevatorMotorKt * elevator(1, k) * kElevatorGearRatio);
    }

    // Arm dynamics with motor
    // Torque balance including gravitational torque on arm
    double gravTorque = kArmMass * kGravity * kArmLength /
                        2.0;  // Approximate as point mass at center
    problem.SubjectTo(kArmMotorKt * armCurrent(0, k) * kArmGearRatio ==
                      kArmMotorJ * armAccel(0, k) * kArmGearRatio *
                              kArmGearRatio +
                          kArmMotorB * arm(1, k) * kArmGearRatio +
                          gravTorque * sleipnir::cos(arm(0, k)));

    // Arm voltage equation
    if (k < N - 1) {
      problem.SubjectTo(armVoltage(0, k) ==
                        armCurrent(0, k) * kArmMotorR +
                            kArmMotorL *
                                (armCurrent(0, k + 1) - armCurrent(0, k)) /
                                dt.count() +
                            kArmMotorKt * arm(1, k) * kArmGearRatio);
    }

    // Motor constraints
    problem.SubjectTo(-kElevatorMaxCurrent <= elevatorCurrent(0, k));
    problem.SubjectTo(elevatorCurrent(0, k) <= kElevatorMaxCurrent);
    problem.SubjectTo(-kElevatorMaxVoltage <= elevatorVoltage(0, k));
    problem.SubjectTo(elevatorVoltage(0, k) <= kElevatorMaxVoltage);

    problem.SubjectTo(-kArmMaxCurrent <= armCurrent(0, k));
    problem.SubjectTo(armCurrent(0, k) <= kArmMaxCurrent);
    problem.SubjectTo(-kArmMaxVoltage <= armVoltage(0, k));
    problem.SubjectTo(armVoltage(0, k) <= kArmMaxVoltage);

    // Kinematic constraints (unchanged)
    problem.SubjectTo(elevator(0, k + 1) ==
                      elevator(0, k) + elevator(1, k) * dt.count() +
                          0.5 * elevatorAccel(0, k) * dt.count() * dt.count());
    problem.SubjectTo(elevator(1, k + 1) ==
                      elevator(1, k) + elevatorAccel(0, k) * dt.count());

    problem.SubjectTo(arm(0, k + 1) ==
                      arm(0, k) + arm(1, k) * dt.count() +
                          0.5 * armAccel(0, k) * dt.count() * dt.count());
    problem.SubjectTo(arm(1, k + 1) == arm(1, k) + armAccel(0, k) * dt.count());
  }

  // Boundary conditions and constraints (unchanged)
  problem.SubjectTo(elevator.Col(0) ==
                    Eigen::Vector2d({kElevatorStartHeight, 0.0}));
  problem.SubjectTo(elevator.Col(N) ==
                    Eigen::Vector2d({kElevatorEndHeight, 0.0}));

  problem.SubjectTo(arm.Col(0) == Eigen::Vector2d({kArmStartAngle, 0.0}));
  problem.SubjectTo(arm.Col(N) == Eigen::Vector2d({kArmEndAngle, 0.0}));

  problem.SubjectTo(-kElevatorMaxVelocity <= elevator.Row(1));
  problem.SubjectTo(elevator.Row(1) <= kElevatorMaxVelocity);

  problem.SubjectTo(-kElevatorMaxAcceleration <= elevatorAccel);
  problem.SubjectTo(elevatorAccel <= kElevatorMaxAcceleration);

  problem.SubjectTo(-kArmMaxVelocity <= arm.Row(1));
  problem.SubjectTo(arm.Row(1) <= kArmMaxVelocity);

  problem.SubjectTo(-kArmMaxAcceleration <= armAccel);
  problem.SubjectTo(armAccel <= kArmMaxAcceleration);

  problem.SubjectTo(elevator.Row(0) +
                        kArmLength * arm.Row(0).CwiseTransform(sleipnir::sin) <=
                    kEndEffectorMaxHeight);

  // Cost function including electrical power for both motors
  sleipnir::Variable J = 0.0;
  for (int k = 0; k < N + 1; ++k) {
    // Original trajectory cost
    J += sleipnir::pow(kElevatorEndHeight - elevator(0, k), 2) +
         sleipnir::pow(kArmEndAngle - arm(0, k), 2);

    // Add electrical power costs
    if (k < N) {
      // I²R losses for both motors
      J += 0.001 * (sleipnir::pow(elevatorCurrent(0, k), 2) * kElevatorMotorR +
                    sleipnir::pow(armCurrent(0, k), 2) * kArmMotorR);
    }
  }
  problem.Minimize(J);

  auto status = problem.Solve({.diagnostics = true});

  bool success =
      status.exitCondition == sleipnir::SolverExitCondition::kSuccess;

  if (success) {
    std::cout << "Found solution!\n";
  } else {
    std::cout << "Failed to find solution.\n";
  }

  return success;
}

void ArmTraj::Periodic() {
  double elevatorPos = elevator(0, currentTimeStep).Value();
  double armPos = arm(0, currentTimeStep).Value();

  elevatorVis->SetLength(elevatorPos);
  armVis->SetAngle(units::radian_t{armPos});

  currentTimeStep++;

  if (currentTimeStep > N) {
    currentTimeStep = 0;
  }
}