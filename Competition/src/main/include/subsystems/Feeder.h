/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"
#include "ValorGamepad.h"

#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>

#include "sensors/ValorCurrentSensor.h"
#include "sensors/ValorDebounceSensor.h"

#ifndef FEEDER_H
#define FEEDER_H

class Feeder : public ValorSubsystem
{
public:
    Feeder();

    void init();
    void setControllers(ValorGamepad *controllerO, ValorGamepad *controllerD);

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    void resetState();

    enum FeederState {
        FEEDER_DISABLE,
        FEEDER_REVERSE,
        FEEDER_SHOOT,
        FEEDER_CURRENT_INTAKE,
        FEEDER_REGULAR_INTAKE,
        FEEDER_RETRACT
    };
    
    struct x
    {

        bool reversed;
        
        double intakeForwardSpeed;
        double intakeReverseSpeed;

        double feederForwardSpeedDefault;
        double feederForwardSpeedShoot;
        double feederReverseSpeed;

        FeederState feederState;
    } state;

    void resetIntakeSensor();

private:
    ValorGamepad *driverController;
    ValorGamepad *operatorController;

    WPI_TalonFX motor_intake;
    WPI_TalonFX motor_stage;
    WPI_TalonFX motor_rotateRight;
    WPI_TalonFX motor_rotateLeft; 
    
    //fixme // create motor group if needed



    ValorCurrentSensor currentSensor;
    ValorDebounceSensor debounceSensor;

    frc::DigitalInput banner;

};

#endif