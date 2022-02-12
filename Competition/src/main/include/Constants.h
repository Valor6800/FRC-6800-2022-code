/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <cmath>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

//CONTROLLER 0 is operator
//CONTROLLER 1 is driver

namespace OIConstants {
    constexpr static int GAMEPAD_BASE_LOCATION = 1;
    constexpr static int GAMEPAD_OPERATOR_LOCATION = 0;

    constexpr static int dpadUp = 0;
    constexpr static int dpadRight = 90;
    constexpr static int dpadDown = 180;
    constexpr static int dpadLeft = 270;
}

namespace LimelightConstants {
    constexpr static int LED_MODE_ON = 3;
    constexpr static int LED_MODE_OFF = 1;
    constexpr static int TRACK_MODE_ON = 0;
    constexpr static int TRACK_MODE_OFF = 1;
}

namespace DriveConstants {
    constexpr static int DRIVE_CANS[4] = {1, 3, 5, 7};
    constexpr static int AZIMUTH_CANS[4] = {2, 4, 6, 8};
    constexpr static int PIGEON_CAN = 61;
    constexpr static int MAG_ENCODER_PORTS[4] = {1, 2, 3, 4};
    constexpr static int MODULE_DIFF_XS[4] = {1, 1, -1, -1}; //{1, 1, -1, -1};
    constexpr static int MODULE_DIFF_YS[4] = {1, -1, 1, -1}; //{-1, 1, -1, 1};

    constexpr static double kDeadbandX = 0.05;
    constexpr static double kDeadbandY = 0.05;

    constexpr static double TURN_KP = 3.8 * M_PI / 180.0;
    constexpr static double LIMELIGHT_KP = .02;

    constexpr static double KPX = .75; //.2
    constexpr static double KIX = 0.0; //0
    constexpr static double KDX = 0.0; //.1

    constexpr static double KPY = .75; //.2
    constexpr static double KIY = 0.0; //0
    constexpr static double KDY = 0.0; //.1


    constexpr static double KPT = 2.5; //.2
    constexpr static double KIT = 0.0; //0
    constexpr static double KDT = 0.0; //.1

}

namespace SwerveConstants {
    constexpr static double module_diff = 0.22225;
    constexpr static auto SWERVE_MODULE_DIFF_X = units::meter_t(module_diff);
    constexpr static auto SWERVE_MODULE_DIFF_Y = units::meter_t(module_diff);

    constexpr static double AZIMUTH_COUNTS_PER_REV = 2048;
    constexpr static double DRIVE_COUNTS_PER_REV = 2048;
    constexpr static double MAG_COUNTS_PER_REV = 4096;

    constexpr static double DRIVE_GEAR_RATIO = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0); // 1/8.14
    constexpr static double AZIMUTH_GEAR_RATIO = (15.0 / 32.0) * (10.0 / 60.0); // 0.078125

    constexpr static double WHEEL_DIAMETER_M = 0.1016;
    constexpr static double WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * M_PI;

    constexpr static double MOTOR_FREE_SPEED = 6380.0;

    constexpr static units::meters_per_second_t DRIVE_DEADBAND_MPS = units::meters_per_second_t{0.05};

    constexpr static double DRIVE_MAX_SPEED_MPS = MOTOR_FREE_SPEED / 60.0 * DRIVE_GEAR_RATIO * WHEEL_DIAMETER_M * M_PI;
    constexpr static double AUTO_MAX_SPEED_MPS = DRIVE_MAX_SPEED_MPS;
    constexpr static double AUTO_MAX_ACCEL_MPSS = AUTO_MAX_SPEED_MPS * 1;

    static double ROTATION_MAX_SPEED_RPS = 2*M_PI;// DRIVE_MAX_SPEED_MPS / std::hypot(module_diff / 2, module_diff / 2);
    static double AUTO_MAX_ROTATION_RPS = 4 * M_PI;
    static double AUTO_MAX_ROTATION_ACCEL_RPSS = AUTO_MAX_ROTATION_RPS * 1;

    constexpr static double MOTION_CRUISE_VELOCITY = 17000; // 21.9k is max. We chose 80%ish of that number
    constexpr static double MOTION_ACCELERATION = MOTION_CRUISE_VELOCITY * 20; // Acceleration matching cruise velocity means 1 second to hit cruise
    constexpr static double KF = 0.05; // Calculated via kF math in the Phoenix documentation
    
    constexpr static double KP = 0.2; //.2
    constexpr static double KI = 0.0; //0
    constexpr static double KD = 0.1; //.1
}

namespace ShooterConstants{
    constexpr static int CAN_ID_FLYWHEEL_FOLLOW = 13;
    constexpr static int CAN_ID_FLYWHEEL_LEAD = 14;
    constexpr static int CAN_ID_TURRET = 12;
    constexpr static int CAN_ID_HOOD = 15;    
    
    constexpr static double limelightTurnKP = 1/24.0;

    constexpr static double flywheelKP = 0.1;
    constexpr static double flywheelKI = 0;
    constexpr static double flywheelKD = 5;
    constexpr static double flywheelKIZ = 0;
    constexpr static double flywheelKFF = 0.05;
    constexpr static double MaxRPM = 6380;

    constexpr static double flywheelCruiseVelo = 20000;
    constexpr static double flywheelMinV = 0;
    constexpr static double flywheelMaxAccel = flywheelCruiseVelo * 1;
    constexpr static double flywheelAllowedError = 0;

    constexpr static double flywheelPrimedValue = .6;
    constexpr static double flywheelDefaultValue = 0.5;

    constexpr static double turretKP = 0;
    constexpr static double turretKI = 0;
    constexpr static double turretKD = 0;
    constexpr static double turretKIZ = 0;
    constexpr static double turretKFF = 0.00023;

    constexpr static double turretMaxV = 2000;
    constexpr static double turretMinV = 0;
    constexpr static double turretMaxAccel = 1500;
    constexpr static double turretAllowedError = 0;

    constexpr static double hoodKP = 0;
    constexpr static double hoodKI = 0;
    constexpr static double hoodKD = 0;
    constexpr static double hoodKIZ = 0;
    constexpr static double hoodKFF = 0.00023;

    constexpr static double hoodMaxV = 2000;
    constexpr static double hoodMinV = 0;
    constexpr static double hoodMaxAccel = 1500;
    constexpr static double hoodAllowedError = 0;

    constexpr static int hoodTop = 1060; //calculated
    constexpr static int hoodBottom = 0;

    constexpr static double kDeadband = .05;

    constexpr static double pDeadband = .01;
    constexpr static double TURRET_SPEED_MULTIPLIER = 1;
    constexpr static double pSoftDeadband = 0.06;

    constexpr static double homePosition = 0;

    constexpr static double falconMaxRPM = 6380;
    constexpr static double falconGearRatio = 1;

    constexpr static double turretGearRatio = 30;

    // Encoder ticks off of center
    // 192 (gear ration) * angle ratio (ex. 1/2 for 180 deg)
    constexpr static double limitLeft = homePosition + 200; // 20;
    constexpr static double limitRight = homePosition - 15; // -15;

    constexpr static double limitTop = homePosition + 50;
    constexpr static double limitBottom = homePosition - 5;

    constexpr static double hubX = 0;
    constexpr static double hubY = 0;

    constexpr static double cornerX = 0;
    constexpr static double cornerY = 0;

    constexpr static double ticsPerRev = 2048;
}

namespace FeederConstants{
    constexpr static int MOTOR_INTAKE_CAN_ID = 9;
    constexpr static int MOTOR_STAGE_CAN_ID = 10;

    constexpr static int BANNER_DIO_PORT = 5;

    constexpr static double DEFAULT_INTAKE_SPEED_FORWARD = 0.9;
    constexpr static double DEFAULT_INTAKE_SPEED_REVERSE = -0.9;

    constexpr static double DEFAULT_FEEDER_SPEED_FORWARD_DEFAULT = 0.5;
    constexpr static double DEFAULT_FEEDER_SPEED_FORWARD_SHOOT = 1.0;
    constexpr static double DEFAULT_FEEDER_SPEED_REVERSE = -1.0;

    constexpr static int CACHE_SIZE = 10;
    constexpr static double JAM_CURRENT = 20;
}

namespace MathConstants{
    constexpr static double toRadians = M_PI / 180.0;
    constexpr static double toDegrees = 180.0 / M_PI;

    constexpr static double ticksToRads = 2.0 * M_PI  / SwerveConstants::AZIMUTH_COUNTS_PER_REV * SwerveConstants::AZIMUTH_GEAR_RATIO;
    constexpr static double radsToTicks = 1 / ticksToRads;
}

namespace LiftConstants{
    constexpr static int MAIN_CAN_ID = 16;
    constexpr static int MAIN_FOLLOW_CAN_ID = 17;
    constexpr static int AUX_CAN_ID = 18;
    constexpr static int ROTATE_CAN_ID = 19;

    constexpr static double DEFAULT_EXTEND_SPD = 0.2;
    constexpr static double DEFAULT_RETRACT_SPD = -0.2;

    constexpr static double DEFAULT_AUX_EXTEND_SPD = 0.2;
    constexpr static double DEFAULT_AUX_RETRACT_SPD = -0.2;

    constexpr static double DEFAULT_ROTATE_SPD = 0.2;

}

#endif
