#pragma once

#include <sleipnir/optimization/OptimizationProblem.hpp>

#include <frc/smartdashboard/Mechanism2d.h>
#include "frc/smartdashboard/MechanismRoot2d.h"
#include <frc/smartdashboard/MechanismLigament2d.h>
#include "frc/smartdashboard/SmartDashboard.h"

namespace str {

class ArmTraj {
 public:
  ArmTraj() { frc::SmartDashboard::PutData("ArmVator", &armvator); };
  int CalculateTraj();
  void Periodic();

 private:
  constexpr static int N = 800;
  int currentTimeStep = 0;
  constexpr static double kArmLength = 0.6096;  // m

  sleipnir::OptimizationProblem problem;

  // Existing mechanical state variables
  sleipnir::VariableMatrix elevator = problem.DecisionVariable(2, N + 1);
  sleipnir::VariableMatrix elevatorAccel = problem.DecisionVariable(1, N);

  sleipnir::VariableMatrix arm = problem.DecisionVariable(2, N + 1);
  sleipnir::VariableMatrix armAccel = problem.DecisionVariable(1, N);

  frc::Mechanism2d armvator{3, 3};
  frc::MechanismRoot2d* root{armvator.GetRoot("root", 1.5, 0)};
  frc::MechanismLigament2d* elevatorVis =
      root->Append<frc::MechanismLigament2d>("elevator", 0, 90_deg);
  frc::MechanismLigament2d* armVis =
      elevatorVis->Append<frc::MechanismLigament2d>("arm", kArmLength, 0_deg);
};
}  // namespace str