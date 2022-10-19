/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/Command.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <vector>
#include "Auto.h"
#include "auto/ValorAuto.h"
#include "Constants.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Shooter.h"
#include "subsystems/Feeder.h"
#include "subsystems/Lift.h"
#include "subsystems/TurretTracker.h"
#include "ValorGamepad.h"
// #include "auto/ValorPoints.h"

#ifndef ROBOT_CONTAINER_H
#define ROBOT_CONTAINER_H

class RobotContainer {
    public:
        RobotContainer();
        frc2::Command* GetAutonomousCommand();

        ValorGamepad m_GamepadDriver{OIConstants::GAMEPAD_BASE_LOCATION};
        ValorGamepad m_GamepadOperator{OIConstants::GAMEPAD_OPERATOR_LOCATION};

        Drivetrain m_drivetrain;
        Shooter m_shooter;
        Feeder m_feeder;
        Lift m_lift;
        TurretTracker m_turretTracker;

    private:
        Auto m_auto;
        ValorAuto testAuto;
        void ConfigureButtonBindings();
        std::map<std::string, frc::Translation2d> points;
};

#endif
 