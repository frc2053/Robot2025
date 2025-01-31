// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <wpi/MemoryBuffer.h>
#include <wpi/json.h>

#include <string>
#include <unordered_map>

#include "ProjectFile.h"
#include "frc/Errors.h"
#include "frc/Filesystem.h"
#include "frc/geometry/Pose3d.h"
#include "units/angle.h"

namespace strchoreo {

static constexpr std::string_view PROJ_FILE_EXTENSION = ".chor";

static inline const std::string CHOREO_DIR =
    frc::filesystem::GetDeployDirectory() + "/choreo";

static std::unordered_map<std::string, frc::Pose2d> LoadPoses() {
  std::string choreoProjectFileName =
      fmt::format("{}/{}{}", CHOREO_DIR, "robotchoreo", PROJ_FILE_EXTENSION);

  auto fileBuffer = wpi::MemoryBuffer::GetFile(choreoProjectFileName);
  if (!fileBuffer) {
    FRC_ReportError(frc::warn::Warning, "Could not find choreo project file");
    return {};
  }

  try {
    std::unordered_map<std::string, frc::Pose2d> result;
    wpi::json json =
        wpi::json::parse(std::string{fileBuffer.value()->GetCharBuffer().data(),
                                     fileBuffer.value()->size()});
    choreo::ProjectFile proj;
    from_json(json, proj);

    for (const auto& [key, choreoPose] : proj.poses) {
      // Convert choreo::Pose to frc::Pose2d
      frc::Translation2d translation2d(units::meter_t{choreoPose.x.val},
                                       units::meter_t{choreoPose.y.val});
      frc::Rotation2d rotation2d(units::radian_t{choreoPose.heading.val});

      // Add to result
      result.emplace(key, frc::Pose2d(translation2d, rotation2d));
    }
    return result;
  } catch (wpi::json::parse_error& ex) {
    FRC_ReportError(frc::warn::Warning, "Could not parse project file!");
    FRC_ReportError(frc::warn::Warning, "{}", ex.what());
    return {};
  }
  return {};
}
}  // namespace strchoreo
