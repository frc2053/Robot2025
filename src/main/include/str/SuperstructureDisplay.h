// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <units/length.h>

#include <memory>

#include "constants/SwerveConstants.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Transform3d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Translation3d.h"
#include "frc/smartdashboard/Mechanism2d.h"
#include "frc/smartdashboard/MechanismLigament2d.h"
#include "frc/smartdashboard/MechanismObject2d.h"
#include "frc/smartdashboard/MechanismRoot2d.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/util/Color.h"
#include "networktables/StructArrayTopic.h"
#include "units/angle.h"

namespace str {
class SuperstructureDisplay {
 public:
  SuperstructureDisplay() = default;

  void Draw() {
    frc::SmartDashboard::PutData("Superstructure Display",
                                 &superstructureDisplay);
    reefMainPole->SetColor(frc::Color::kPink);
    reefL2->SetColor(frc::Color::kPink);
    l2Pole->SetColor(frc::Color::kPink);
    reefL3->SetColor(frc::Color::kPink);
    l3Pole->SetColor(frc::Color::kPink);
    l4DiagPole->SetColor(frc::Color::kPink);
    reefL4->SetColor(frc::Color::kPink);
    reefBottom->SetColor(frc::Color::kGray);
    reefFrontPlate->SetColor(frc::Color::kGray);
    coralStationRamp->SetColor(frc::Color::kGray);

    reefMainPole->SetLineWeight(PIPE_THICKNESS.value());
    reefL2->SetLineWeight(PIPE_THICKNESS.value());
    l2Pole->SetLineWeight(PIPE_THICKNESS.value());
    reefL3->SetLineWeight(PIPE_THICKNESS.value());
    l3Pole->SetLineWeight(PIPE_THICKNESS.value());
    l4DiagPole->SetLineWeight(PIPE_THICKNESS.value());
    reefL4->SetLineWeight(PIPE_THICKNESS.value());
    reefBottom->SetLineWeight(PIPE_THICKNESS.value());
    reefFrontPlate->SetLineWeight(PIPE_THICKNESS.value());
    coralStationRamp->SetLineWeight(PIPE_THICKNESS.value());

    drivebaseBottomLeft->SetColor(frc::Color::kYellow);
    drivebaseBottomRight->SetColor(frc::Color::kYellow);
    drivebaseLeft->SetColor(frc::Color::kYellow);
    drivebaseRight->SetColor(frc::Color::kYellow);
    drivebaseTop->SetColor(frc::Color::kYellow);

    elevatorCarriage->SetColor(frc::Color::kRed);

    pivotCgVis->SetColor(frc::Color::kCyan);
    pivotJoint->SetColor(frc::Color::kBlue);
    uShapeBack->SetColor(frc::Color::kOrange);
    uShapeAround->SetColor(frc::Color::kOrange);

    algaeIntakeUpright->SetColor(frc::Color::kDarkGreen);
    algaeIntakeArmFirstPart->SetColor(frc::Color::kGreen);
    algaeIntakeArmSecondPart->SetColor(frc::Color::kGreen);
  }
  void SetRobotPose(frc::Pose2d robotPose) { currentPose = robotPose; }
  void SetElevatorHeight(units::meter_t newHeight) {
    elevatorCarriage->SetLength(newHeight / 1_m);
    superstructurePoses[0] =
        frc::Pose3d{0_m, 0_m, newHeight / 3.0, frc::Rotation3d{}};
    superstructurePoses[1] =
        frc::Pose3d{0_m, 0_m, (newHeight / 3.0) * 2, frc::Rotation3d{}};
    superstructurePoses[2] =
        frc::Pose3d{0_m, 0_m, newHeight, frc::Rotation3d{}};
    aScopeDisplay.Set(superstructurePoses);
  }
  void SetPivotAngle(units::radian_t newAngle) {
    pivotCgVis->SetAngle(newAngle - 90_deg);
    pivotJoint->SetAngle(newAngle - 90_deg + 55.7_deg);
    uShapeBack->SetAngle(newAngle - 90_deg + 55.7_deg - 116.16_deg);
    currentAngle = (newAngle - 90_deg + 55.7_deg);

    superstructurePoses[3] = iHonestlyForgetWhatThisRepresents.TransformBy(
        frc::Transform3d{superstructurePoses[2].Translation(),
                         frc::Rotation3d{currentAngle, 0_rad, 0_rad}});
  }

  void SetAlgaeIntakeAngle(units::radian_t newAngle) {
    algaeIntakeArmFirstPart->SetAngle(-(newAngle - 90_deg));

    superstructurePoses[4] =
        frc::Pose3d{-0.3048_m, 0.11_m, 0.530225_m, frc::Rotation3d{}}
            .TransformBy(frc::Transform3d{
                frc::Translation3d{}, frc::Rotation3d{0_rad, newAngle, 0_rad}});
  }

  void AlgaeIntakeGamePiece(bool hasAlgae) {
    if (hasAlgae) {
      algae[1] = frc::Pose3d{frc::Translation3d{currentPose.Translation()},
                             frc::Rotation3d{currentPose.Rotation()}}
                     .TransformBy(frc::Transform3d{-20_in, 0_in, 12_in,
                                                   frc::Rotation3d{}});
    } else {
      algae[1] = frc::Pose3d{};
    }
    aScopeAlgae.Set(algae);
  }

  void GamePieceSet(bool hasCoral, bool hasAlgae) {
    if (hasAlgae) {
      algae[0] = frc::Pose3d{frc::Translation3d{currentPose.Translation()},
                             frc::Rotation3d{currentPose.Rotation()}}
                     .TransformBy(frc::Transform3d{
                         frc::Translation3d{superstructurePoses[3].X(),
                                            superstructurePoses[3].Y(),
                                            superstructurePoses[3].Z()},
                         superstructurePoses[3].Rotation()})
                     .TransformBy(
                         frc::Transform3d{0_in, 8_in, 6_in, frc::Rotation3d{}});
    } else {
      algae[0] = frc::Pose3d{};
    }
    if (hasCoral) {
      coral[0] =
          frc::Pose3d{frc::Translation3d{currentPose.Translation()},
                      frc::Rotation3d{currentPose.Rotation()}}
              .TransformBy(frc::Transform3d{
                  frc::Translation3d{superstructurePoses[3].X(),
                                     superstructurePoses[3].Y(),
                                     superstructurePoses[3].Z()},
                  superstructurePoses[3].Rotation()})
              .TransformBy(frc::Transform3d{
                  0_in, 0_in, 11_in, frc::Rotation3d{0_deg, 100_deg, 90_deg}});
    } else {
      coral[0] = frc::Pose3d{};
    }
    aScopeCoral.Set(coral);
    aScopeAlgae.Set(algae);
  }

 private:
  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("3DDisplay")};
  nt::StructArrayPublisher<frc::Pose3d> aScopeDisplay{
      nt->GetStructArrayTopic<frc::Pose3d>("Superstructure").Publish()};
  nt::StructArrayPublisher<frc::Pose3d> aScopeCoral{
      nt->GetStructArrayTopic<frc::Pose3d>("Coral").Publish()};
  nt::StructArrayPublisher<frc::Pose3d> aScopeAlgae{
      nt->GetStructArrayTopic<frc::Pose3d>("Algae").Publish()};
  std::array<frc::Pose3d, 5> superstructurePoses{frc::Pose3d{}, frc::Pose3d{},
                                                 frc::Pose3d{}, frc::Pose3d{},
                                                 frc::Pose3d{}};
  std::array<frc::Pose3d, 1> coral{frc::Pose3d{}};
  std::array<frc::Pose3d, 2> algae{frc::Pose3d{}, frc::Pose3d{}};

  frc::Pose2d currentPose{};
  units::radian_t currentAngle{};

  frc::Pose3d iHonestlyForgetWhatThisRepresents =
      frc::Pose3d{frc::Translation3d{0.1016508_m, -0.117274594_m, 0.263525_m},
                  frc::Rotation3d{0_deg, 0_deg, -90_deg}};

  inline static constexpr units::meter_t TOTAL_SCREEN_WIDTH = 60_in;
  inline static constexpr units::meter_t TOTAL_SCREEN_HEIGHT = 120_in;
  inline static constexpr units::meter_t PIPE_THICKNESS = 1.66_in;
  inline static constexpr units::meter_t TOP_OF_SWERVE = 6.062500_in;
  inline static constexpr units::meter_t MIDDLE_OF_SWERVE =
      TOTAL_SCREEN_WIDTH / 2;
  inline static constexpr units::meter_t MIDDLE_OF_CARRIAGE =
      MIDDLE_OF_SWERVE - 4_in;
  inline static constexpr units::meter_t LOWEST_CARRIAGE_HEIGHT = 9.5000_in;

  inline static constexpr units::meter_t ALGAE_INTAKE =
      MIDDLE_OF_SWERVE + 12_in;
  inline static constexpr units::meter_t ALGAE_INTAKE_UPRIGHT = 14.812851_in;
  inline static constexpr units::meter_t ALGAE_ARM_FIRST_PART = 7.193467_in;
  inline static constexpr units::meter_t ALGAE_ARM_SECOND_PART = 6.534639_in;

  frc::Mechanism2d superstructureDisplay{TOTAL_SCREEN_WIDTH.value(),
                                         TOTAL_SCREEN_HEIGHT.value()};

  // Reef L2-L4
  frc::MechanismRoot2d* reef{superstructureDisplay.GetRoot(
      "Reef", TOTAL_SCREEN_WIDTH.value() - (PIPE_THICKNESS.value() / 2), 0)};
  frc::MechanismLigament2d* reefMainPole{reef->Append<frc::MechanismLigament2d>(
      "MainPole", 24.149027_in / 1_m, 90_deg)};
  frc::MechanismLigament2d* reefL2{
      reefMainPole->Append<frc::MechanismLigament2d>("L2", 12.119637_in / 1_m,
                                                     55_deg)};
  frc::MechanismLigament2d* l2Pole{
      reefMainPole->Append<frc::MechanismLigament2d>("L2Pole", 15.84_in / 1_m,
                                                     0_deg)};
  frc::MechanismLigament2d* reefL3{l2Pole->Append<frc::MechanismLigament2d>(
      "L3", 12.119637_in / 1_m, 55_deg)};
  frc::MechanismLigament2d* l3Pole{l2Pole->Append<frc::MechanismLigament2d>(
      "L3Pole", 15.536492_in / 1_m, 0_deg)};
  frc::MechanismLigament2d* l4DiagPole{l3Pole->Append<frc::MechanismLigament2d>(
      "L4DiagPole", 12.119637_in / 1_m, 55_deg)};
  frc::MechanismLigament2d* reefL4{l4DiagPole->Append<frc::MechanismLigament2d>(
      "L4", 9.810891_in / 1_m, -55_deg)};

  // Reef L1 Parts
  frc::MechanismLigament2d* reefBottom{reef->Append<frc::MechanismLigament2d>(
      "ReefBottom", 12.052349_in / 1_m, 180_deg)};
  frc::MechanismLigament2d* reefFrontPlate{
      reefBottom->Append<frc::MechanismLigament2d>(
          "ReefFrontPlate", 17.875000_in / 1_m, -90_deg)};

  // Loading Station
  frc::MechanismRoot2d* coralStation{superstructureDisplay.GetRoot(
      "CoralStation", (2_in / 1_m), 37.440179_in / 1_m)};
  frc::MechanismLigament2d* coralStationRamp{
      coralStation->Append<frc::MechanismLigament2d>("CoralRamp", 2_in / 1_m,
                                                     -35_deg)};

  // Drivebase
  frc::MechanismRoot2d* drivebaseCenter{superstructureDisplay.GetRoot(
      "Robot", TOTAL_SCREEN_WIDTH.value() - MIDDLE_OF_SWERVE.value(), 0)};
  frc::MechanismLigament2d* drivebaseBottomLeft{
      drivebaseCenter->Append<frc::MechanismLigament2d>(
          "DTBottomLeft", (consts::swerve::physical::TOTAL_LENGTH / 2).value(),
          0_deg)};
  frc::MechanismLigament2d* drivebaseBottomRight{
      drivebaseCenter->Append<frc::MechanismLigament2d>(
          "DTBottomRight", (consts::swerve::physical::TOTAL_LENGTH / 2).value(),
          180_deg)};
  frc::MechanismLigament2d* drivebaseLeft{
      drivebaseBottomLeft->Append<frc::MechanismLigament2d>(
          "DTLeft", TOP_OF_SWERVE.value(), 90_deg)};
  frc::MechanismLigament2d* drivebaseTop{
      drivebaseLeft->Append<frc::MechanismLigament2d>(
          "DTTop", consts::swerve::physical::TOTAL_LENGTH.value(), 90_deg)};
  frc::MechanismLigament2d* drivebaseRight{
      drivebaseBottomRight->Append<frc::MechanismLigament2d>(
          "DTRight", TOP_OF_SWERVE.value(), -90_deg)};

  // ALGAE INTAKE
  frc::MechanismRoot2d* algaeIntakeRoot{superstructureDisplay.GetRoot(
      "AlgaeIntakeRoot", TOTAL_SCREEN_WIDTH.value() - ALGAE_INTAKE.value(),
      TOP_OF_SWERVE.value())};
  frc::MechanismLigament2d* algaeIntakeUpright{
      algaeIntakeRoot->Append<frc::MechanismLigament2d>(
          "AlgaeIntakeUpright", ALGAE_INTAKE_UPRIGHT.value(), 90_deg)};
  frc::MechanismLigament2d* algaeIntakeArmFirstPart{
      algaeIntakeUpright->Append<frc::MechanismLigament2d>(
          "AlgaeIntakeArmFirst", ALGAE_ARM_FIRST_PART.value(), -109.5_deg)};
  frc::MechanismLigament2d* algaeIntakeArmSecondPart{
      algaeIntakeArmFirstPart->Append<frc::MechanismLigament2d>(
          "AlgaeIntakeArmSecond", ALGAE_ARM_SECOND_PART.value(), 10.5_deg)};

  // Elevator
  frc::MechanismRoot2d* elevatorRoot{superstructureDisplay.GetRoot(
      "ElevatorRoot", TOTAL_SCREEN_WIDTH.value() - MIDDLE_OF_CARRIAGE.value(),
      LOWEST_CARRIAGE_HEIGHT.value())};
  frc::MechanismLigament2d* elevatorCarriage{
      elevatorRoot->Append<frc::MechanismLigament2d>("ElevatorCarriage", 0,
                                                     90_deg)};

  // Pivot
  frc::MechanismLigament2d* pivotCgVis{
      elevatorCarriage->Append<frc::MechanismLigament2d>(
          "PivotCgVis", 4.37480954214_in / 1_m, 0_deg)};
  frc::MechanismLigament2d* pivotJoint{
      elevatorCarriage->Append<frc::MechanismLigament2d>(
          "PivotJoint", 9.508097_in / 1_m, 0_deg)};
  frc::MechanismLigament2d* uShapeBack{
      elevatorCarriage->Append<frc::MechanismLigament2d>(
          "uShapeBack", 9.508097_in / 1_m, 0_deg)};
  frc::MechanismLigament2d* uShapeAround{
      uShapeBack->Append<frc::MechanismLigament2d>(
          "uShapeAround", 13.808011_in / 1_m, 66.28_deg)};
};
}  // namespace str
