// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc/system/plant/DCMotor.h>
#include <str/GainTypes.h>
#include <str/Units.h>
#include <units/angle.h>
#include <units/current.h>
#include <units/dimensionless.h>
#include <units/frequency.h>
#include <units/length.h>
#include <units/voltage.h>

#include <string>
#include <string_view>

#include "units/angular_velocity.h"
#include "units/velocity.h"

namespace str::swerve {
struct ModuleConstants {
  const std::string moduleName;
  const int driveId;
  const int steerId;
  const int encoderId;
  const units::turn_t steerEncoderOffset;
  const bool invertDrive;
  const bool invertSteer;

  ModuleConstants() = delete;
  ModuleConstants(std::string_view name, int dId, int sId, int eId,
                  units::turn_t offset, bool invDrive, bool invSteer)
      : moduleName(name),
        driveId(dId),
        steerId(sId),
        encoderId(eId),
        steerEncoderOffset(offset),
        invertDrive(invDrive),
        invertSteer(invSteer) {}
};

struct ModulePhysicalCharacteristics {
  const units::scalar_t steerGearing;
  const units::scalar_t driveGearing;
  const units::ampere_t steerSupplySideLimit;
  const units::ampere_t driveSupplySideLimit;
  const units::ampere_t steerStatorCurrentLimit;
  const units::ampere_t driveStatorCurrentLimit;
  const frc::DCMotor steerMotor;
  const frc::DCMotor driveMotor;
  const units::scalar_t couplingRatio;
  const units::meter_t wheelRadius;
  // Used for sim only
  const units::volt_t driveFrictionVoltage{0.25_V};
  const units::volt_t steerFrictionVoltage{0.25_V};

  const units::meters_per_second_t MaxLinearSpeed() const {
    return ((driveMotor.freeSpeed / 1_rad) / driveGearing) * wheelRadius;
  }

  ModulePhysicalCharacteristics() = delete;
  ModulePhysicalCharacteristics(
      units::scalar_t steerGear, units::scalar_t driveGear,
      units::ampere_t steerSupplyLim, units::ampere_t driveSupplyLim,
      units::ampere_t steerStatorLim, units::ampere_t driveStatorLim,
      const frc::DCMotor& steer, const frc::DCMotor drive,
      units::scalar_t coupling, units::meter_t wheelRad)
      : steerGearing{steerGear},
        driveGearing{driveGear},
        steerSupplySideLimit{steerSupplyLim},
        driveSupplySideLimit{driveSupplyLim},
        steerStatorCurrentLimit{steerStatorLim},
        driveStatorCurrentLimit{driveStatorLim},
        steerMotor{steer},
        driveMotor{drive},
        couplingRatio{coupling},
        wheelRadius{wheelRad} {}
};

struct DriveGains {
  str::gains::radial::turn_amp_ka_unit_t kA;
  str::gains::radial::turn_amp_kv_unit_t kV;
  units::ampere_t kS;
  str::gains::radial::turn_amp_kp_unit_t kP;
  str::gains::radial::turn_amp_ki_unit_t kI;
  str::gains::radial::turn_amp_kd_unit_t kD;

  DriveGains& operator=(const DriveGains& other) = default;
  DriveGains(const DriveGains& other)
      : kA{other.kA},
        kV{other.kV},
        kS{other.kS},
        kP{other.kP},
        kI{other.kI},
        kD{other.kD} {}
  DriveGains(str::gains::radial::turn_amp_ka_unit_t ka,
             str::gains::radial::turn_amp_kv_unit_t kv, units::ampere_t ks,
             str::gains::radial::turn_amp_kp_unit_t kp,
             str::gains::radial::turn_amp_ki_unit_t ki,
             str::gains::radial::turn_amp_kd_unit_t kd)
      : kA{ka}, kV{kv}, kS{ks}, kP{kp}, kI{ki}, kD{kd} {}

  bool operator==(const DriveGains& rhs) const {
    return units::essentiallyEqual(kA, rhs.kA, 1e-6) &&
           units::essentiallyEqual(kV, rhs.kV, 1e-6) &&
           units::essentiallyEqual(kS, rhs.kS, 1e-6) &&
           units::essentiallyEqual(kP, rhs.kP, 1e-6) &&
           units::essentiallyEqual(kI, rhs.kI, 1e-6) &&
           units::essentiallyEqual(kD, rhs.kD, 1e-6);
  }
  bool operator!=(const DriveGains& rhs) const { return !operator==(rhs); }
};

struct WheelRadiusCharData {
  units::radian_t lastGyroYaw;
  units::radian_t accumGyroYaw;
  std::array<units::radian_t, 4> startWheelPositions;
  units::meter_t effectiveWheelRadius = 0_m;
  frc::SlewRateLimiter<units::radians_per_second> omegaLimiter{1_rad_per_s /
                                                               1_s};
};
}  // namespace str::swerve
