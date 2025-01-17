// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <units/angular_acceleration.h>
#include <units/frequency.h>

#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/system/plant/DCMotor.h"
#include "pathplanner/lib/config/RobotConfig.h"
#include "str/swerve/SwerveModuleHelpers.h"
#include "units/angle.h"

namespace consts::swerve {

inline constexpr units::hertz_t ODOM_UPDATE_RATE = 250_Hz;

namespace can_ids {
inline constexpr int FL_DRIVE = 2;
inline constexpr int FL_STEER = 3;
inline constexpr int FL_ENC = 4;

inline constexpr int FR_DRIVE = 5;
inline constexpr int FR_STEER = 6;
inline constexpr int FR_ENC = 7;

inline constexpr int BL_DRIVE = 8;
inline constexpr int BL_STEER = 9;
inline constexpr int BL_ENC = 10;

inline constexpr int BR_DRIVE = 11;
inline constexpr int BR_STEER = 12;
inline constexpr int BR_ENC = 13;

inline constexpr int IMU = 14;
}  // namespace can_ids

namespace current_limits {
inline constexpr units::ampere_t STEER_SUPPLY_LIMIT = 40_A;
inline constexpr units::ampere_t STEER_STATOR_LIMIT = 40_A;
inline constexpr units::ampere_t DRIVE_SUPPLY_LIMIT = 60_A;
inline constexpr units::ampere_t DRIVE_STATOR_LIMIT = 60_A;
}  // namespace current_limits

namespace physical {
inline constexpr frc::DCMotor DRIVE_MOTOR = frc::DCMotor::KrakenX60FOC(1);
inline constexpr frc::DCMotor STEER_MOTOR = frc::DCMotor::Falcon500FOC(1);

inline constexpr units::scalar_t STEER_GEARING = (50.0 / 14.0) * (60.0 / 10.0);
// L2
inline constexpr units::scalar_t DRIVE_GEARING =
    (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
inline constexpr units::scalar_t COUPLING_RATIO = (50.0 / 14.0);
inline constexpr units::meter_t WHEEL_RADIUS = 1.9154_in;

inline constexpr units::meter_t WHEELBASE_WIDTH = 22.75_in;
inline constexpr units::meter_t WHEELBASE_LENGTH = 22.75_in;

inline const units::meter_t DRIVEBASE_RADIUS{
    units::math::hypot(WHEELBASE_WIDTH / 2, WHEELBASE_LENGTH / 2)};

inline constexpr std::array<frc::Translation2d, 4> MODULE_LOCATIONS{
    frc::Translation2d{WHEELBASE_LENGTH / 2, WHEELBASE_WIDTH / 2},
    frc::Translation2d{WHEELBASE_LENGTH / 2, -WHEELBASE_WIDTH / 2},
    frc::Translation2d{-WHEELBASE_LENGTH / 2, WHEELBASE_WIDTH / 2},
    frc::Translation2d{-WHEELBASE_LENGTH / 2, -WHEELBASE_WIDTH / 2}};

inline frc::SwerveDriveKinematics<4> KINEMATICS{
    MODULE_LOCATIONS[0], MODULE_LOCATIONS[1], MODULE_LOCATIONS[2],
    MODULE_LOCATIONS[3]};

// Total outside frame size
inline constexpr units::meter_t DRIVEBASE_WIDTH = 28.44_in;
inline constexpr units::meter_t DRIVEBASE_LENGTH = 28.44_in;

// 3/4 in plywood + 2.5 in diameter pool noodles + 1/8 slop
inline constexpr units::meter_t BUMPER_THICKNESS = .75_in + 2.5_in + .125_in;

// Total size including bumpers
inline constexpr units::meter_t TOTAL_WIDTH =
    DRIVEBASE_WIDTH + (2 * BUMPER_THICKNESS);
inline constexpr units::meter_t TOTAL_LENGTH =
    DRIVEBASE_LENGTH + (2 * BUMPER_THICKNESS);

inline constexpr units::degree_t IMU_MOUNT_ROLL = 0_deg;
inline constexpr units::degree_t IMU_MOUNT_PITCH = 0_deg;
inline constexpr units::degree_t IMU_MOUNT_YAW = 0_deg;

inline constexpr units::turn_t FL_ENC_OFFSET = -0.272949_tr;
inline constexpr units::turn_t FR_ENC_OFFSET = 0.356201_tr;
inline constexpr units::turn_t BL_ENC_OFFSET = .195068_tr;
inline constexpr units::turn_t BR_ENC_OFFSET = -0.492676_tr;

inline constexpr bool FL_STEER_INVERT = true;
inline constexpr bool FR_STEER_INVERT = true;
inline constexpr bool BL_STEER_INVERT = true;
inline constexpr bool BR_STEER_INVERT = true;

inline constexpr bool FL_DRIVE_INVERT = false;
inline constexpr bool FR_DRIVE_INVERT = true;
inline constexpr bool BL_DRIVE_INVERT = false;
inline constexpr bool BR_DRIVE_INVERT = true;

inline const str::swerve::ModuleConstants FL{"FL",
                                             consts::swerve::can_ids::FL_DRIVE,
                                             consts::swerve::can_ids::FL_STEER,
                                             consts::swerve::can_ids::FL_ENC,
                                             FL_ENC_OFFSET,
                                             FL_DRIVE_INVERT,
                                             FL_STEER_INVERT};
inline const str::swerve::ModuleConstants FR{"FR",
                                             consts::swerve::can_ids::FR_DRIVE,
                                             consts::swerve::can_ids::FR_STEER,
                                             consts::swerve::can_ids::FR_ENC,
                                             FR_ENC_OFFSET,
                                             FR_DRIVE_INVERT,
                                             FR_STEER_INVERT};
inline const str::swerve::ModuleConstants BL{"BL",
                                             consts::swerve::can_ids::BL_DRIVE,
                                             consts::swerve::can_ids::BL_STEER,
                                             consts::swerve::can_ids::BL_ENC,
                                             BL_ENC_OFFSET,
                                             BL_DRIVE_INVERT,
                                             BL_STEER_INVERT};
inline const str::swerve::ModuleConstants BR{"BR",
                                             consts::swerve::can_ids::BR_DRIVE,
                                             consts::swerve::can_ids::BR_STEER,
                                             consts::swerve::can_ids::BR_ENC,
                                             BR_ENC_OFFSET,
                                             BR_DRIVE_INVERT,
                                             BR_STEER_INVERT};

inline const str::swerve::ModulePhysicalCharacteristics PHY_CHAR{
    consts::swerve::physical::STEER_GEARING,
    consts::swerve::physical::DRIVE_GEARING,
    consts::swerve::current_limits::STEER_SUPPLY_LIMIT,
    consts::swerve::current_limits::DRIVE_SUPPLY_LIMIT,
    consts::swerve::current_limits::STEER_STATOR_LIMIT,
    consts::swerve::current_limits::DRIVE_STATOR_LIMIT,
    consts::swerve::physical::STEER_MOTOR,
    consts::swerve::physical::DRIVE_MOTOR,
    consts::swerve::physical::COUPLING_RATIO,
    consts::swerve::physical::WHEEL_RADIUS};

inline constexpr units::radians_per_second_t MAX_ROT_SPEED = 540_deg_per_s;
inline constexpr units::radians_per_second_squared_t MAX_ROT_ACCEL =
    720_deg_per_s_sq;
inline constexpr units::meters_per_second_squared_t MAX_ACCEL = 2000_fps_sq;

}  // namespace physical

namespace gains {
inline const str::gains::radial::AmpRadialGainsHolder STEER{
    consts::swerve::physical::STEER_MOTOR.freeSpeed /
        consts::swerve::physical::STEER_GEARING,
    str::gains::radial::turn_volt_ka_unit_t{0.017218},
    str::gains::radial::turn_volt_kv_unit_t{0},
    str::gains::radial::turn_amp_ka_unit_t{0.70949},
    str::gains::radial::turn_amp_kv_unit_t{0},
    9.7875_A,
    str::gains::radial::turn_amp_kp_unit_t{250},
    str::gains::radial::turn_amp_ki_unit_t{0},
    str::gains::radial::turn_amp_kd_unit_t{40},
};

inline const str::swerve::DriveGains DRIVE{
    str::gains::radial::turn_amp_ka_unit_t{0.070827},
    str::gains::radial::turn_amp_kv_unit_t{0},
    10.051_A,
    str::gains::radial::turn_amp_kp_unit_t{3.596},
    str::gains::radial::turn_amp_ki_unit_t{0},
    str::gains::radial::turn_amp_kd_unit_t{0},
};
}  // namespace gains

namespace pathplanning {

inline constexpr units::scalar_t POSE_P = 5;
inline constexpr units::scalar_t POSE_I = 0;
inline constexpr units::scalar_t POSE_D = 0;

inline constexpr units::scalar_t ROTATION_P = 5;
inline constexpr units::scalar_t ROTATION_I = 0;
inline constexpr units::scalar_t ROTATION_D = 0;

// Choreo paths don't support replanning, so just disable me
inline constexpr bool INITIAL_REPLAN = false;
inline constexpr bool DYNAMIC_REPLAN = false;
inline constexpr units::meter_t DYNAMIC_REPLAN_THRESHOLD_TOTAL = 3_ft;
inline constexpr units::meter_t DYNAMIC_REPLAN_THRESHOLD_SPIKE = 1_ft;

inline static pathplanner::RobotConfig config =
    pathplanner::RobotConfig::fromGUISettings();

inline constexpr units::meter_t translationalPIDTolerance = .5_in;
inline constexpr units::meters_per_second_t translationalVelPIDTolerance =
    .25_fps;
inline constexpr units::radian_t rotationalPIDTolerance = 1_deg;
inline constexpr units::radians_per_second_t rotationalVelPIDTolerance =
    1_deg_per_s;
inline constexpr units::meters_per_second_t translationalVelPIDDeadband =
    0.1_fps;
inline constexpr units::radians_per_second_t rotationalVelPIDDeadband =
    0.5_deg_per_s;
}  // namespace pathplanning
}  // namespace consts::swerve
