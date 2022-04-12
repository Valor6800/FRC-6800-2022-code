/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"
#include <deque>

#include <frc/XboxController.h>
#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>
#include <rev/ColorSensorV3.h>
#include "Shooter.h"

#include <queue>

#ifndef FEEDER_H
#define FEEDER_H

class Feeder : public ValorSubsystem
{
public:
    Feeder();

    void init();
    void setControllers(frc::XboxController *controllerO, frc::XboxController *controllerD);

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    void resetState();
    void setShooter(Shooter *s);

    enum FeederState {
        FEEDER_DISABLE,
        FEEDER_REVERSE,
        FEEDER_SHOOT,
        FEEDER_CURRENT_INTAKE,
        FEEDER_REGULAR_INTAKE,
        FEEDER_FEEDER_ONLY
    };
    
    struct x
    {
        bool driver_rightBumperPressed;

        bool operator_bButtonPressed;
        bool operator_aButtonPressed;

        bool driver_leftBumperPressed;
        bool operator_leftBumperPressed;

        bool driver_rightTriggerPressed;
        bool driver_leftTriggerPressed;

        bool bannerTripped;
        bool previousBanner;
        bool currentBanner;

        bool reversed;
        bool spiked;
        
        double intakeForwardSpeed;
        double intakeReverseSpeed;
        double spikeCurrent;

        double feederForwardSpeedDefault;
        double feederForwardSpeedShoot;
        double feederReverseSpeed;

        double blueThreshold;
        double redThreshold;

        bool timing;
        bool autoPoopEnabled;

        int curBall; //0 is nothing, 1 is red, 2 is blue
        int prevBall;

        int counter;
        
        //int current_cache_index;
        //std::vector<double> current_cache;
        std::deque<double> current_cache;

        bool isRedAlliance;

        double instCurrent;

        FeederState feederState;

        std::queue<bool> ballHistory;

    } state;

void resetDeque();

private:
    frc::XboxController *driverController;
    frc::XboxController *operatorController;

    WPI_TalonFX motor_intake;
    WPI_TalonFX motor_stage;

    frc::DigitalInput banner;
    rev::ColorSensorV3 revColorSensor;
    
    void calcCurrent();
    void updateQueue();
    
    bool isRed();
    bool isBlue();
    bool isOppositeColor();

    std::shared_ptr<nt::NetworkTable> fmsTable;
    std::shared_ptr<nt::NetworkTable> limeTable;

    Shooter *shooter;
};

#endif