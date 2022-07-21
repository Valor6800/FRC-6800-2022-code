/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//start button pulls swerve 0 positions from file
//back button pushes current swerve positions to file


#include "subsystems/Feeder.h"
#include <iostream>

#define INTAKE_SPD_FORWARD 0.7
#define INTAKE_SPD_REVERSE -0.7
#define FEEDER_SPD_FORWARD 0.5
#define FEEDER_SPD_SHOOT   0.9
#define FEEDER_SPD_REVERSE -1.0

#define CACHE_SIZE 20
#define JAM_CURRENT 22

Feeder::Feeder() : ValorSubsystem(),
                           driverController(NULL),
                           operatorController(NULL),
                           intakeController(CANIDs::INTAKE, Coast, false, "baseCAN"),
                           stageController(CANIDs::STAGE, Coast, true, "baseCAN"),
                           banner(DIOPorts::BANNER)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Feeder::init()
{
    initTable("Feeder");

    table->PutBoolean("Reverse Feeder?", false);
    table->PutNumber("Intake Reverse Speed", INTAKE_SPD_REVERSE);
    table->PutNumber("Intake Forward Speed", INTAKE_SPD_FORWARD);
    table->PutNumber("Intake Spike Current", JAM_CURRENT);
    table->PutNumber("Feeder Reverse Speed", FEEDER_SPD_REVERSE);
    table->PutNumber("Feeder Forward Speed Default", FEEDER_SPD_FORWARD);
    table->PutNumber("Feeder Forward Speed Shoot", FEEDER_SPD_SHOOT);

    table->PutNumber("Average Amps", 0);
    table->PutBoolean("Spiked: ", 0);
    table->PutBoolean("Banner: ", 0);
    
    resetState();
    currentSensor.setGetter([this]() { return intakeController.getCurrent(); });
    debounceSensor.setGetter([this]() { return !banner.Get(); });
    
}

void Feeder::resetIntakeSensor()
{
    currentSensor.reset();
}

void Feeder::setControllers(ValorGamepad *controllerO, ValorGamepad *controllerD)
{
    driverController = controllerD;
    operatorController = controllerO;
}

void Feeder::assessInputs()
{
    if (!driverController)
    {
        return;
    }
        
    if (driverController->rightTrigger() || operatorController->GetLeftBumper()) {
        state.feederState = FeederState::FEEDER_SHOOT; //intake and feeder run
        resetIntakeSensor();
    }
    else if (driverController->GetLeftBumper()) {
        state.feederState = FeederState::FEEDER_REVERSE;
        resetIntakeSensor();
    }
    else if (driverController->GetRightBumper()) {
        state.feederState = FeederState::FEEDER_REGULAR_INTAKE; //standard intake
    }
    else {
        state.feederState = FeederState::FEEDER_DISABLE;
    }
}

void Feeder::analyzeDashboard()
{
    // Calculate instantaneous current
    currentSensor.calculate();
    // Calculate banner sensor trigger
    debounceSensor.calculate();

    state.reversed = table->GetBoolean("Reverse Feeder?", false);

    state.intakeReverseSpeed = table->GetNumber("Intake Reverse Speed", INTAKE_SPD_REVERSE);
    state.intakeForwardSpeed = table->GetNumber("Intake Forward Speed", INTAKE_SPD_FORWARD);
    state.feederReverseSpeed = table->GetNumber("Feeder Reverse Speed", FEEDER_SPD_REVERSE);
    state.feederForwardSpeedDefault = table->GetNumber("Feeder Forward Speed Default", FEEDER_SPD_FORWARD);
    state.feederForwardSpeedShoot = table->GetNumber("Feeder Forward Speed Shoot", FEEDER_SPD_SHOOT);

    table->PutNumber("Average Amps", currentSensor.getSensor());
    table->PutBoolean("Spiked: ", currentSensor.spiked());
    table->PutBoolean("Banner: ", debounceSensor.getSensor());
    table->PutNumber("current feeder state", state.feederState);
}

void Feeder::assignOutputs()
{

    if (state.feederState == FeederState::FEEDER_DISABLE) {
        intakeController.setPower(0);
        stageController.setPower(0);
    }
    else if (state.feederState == FeederState::FEEDER_SHOOT) {
        intakeController.setPower(state.intakeForwardSpeed);
        stageController.setPower(state.feederForwardSpeedShoot);
    }
    else if (state.feederState == Feeder::FEEDER_REVERSE) {
        intakeController.setPower(state.intakeReverseSpeed);
        stageController.setPower(state.feederReverseSpeed);
    }
    else if (state.feederState == Feeder::FEEDER_REGULAR_INTAKE){
        intakeController.setPower(state.intakeForwardSpeed);
        stageController.setPower(0);
    }
    else if (state.feederState == FeederState::FEEDER_CURRENT_INTAKE) { //includes currrent sensing
        if (debounceSensor.getSensor()) {
            if (debounceSensor.spiked()) {
                resetIntakeSensor();
            }
            if (currentSensor.spiked()) {
                intakeController.setPower(0);
                stageController.setPower(0);
            }
            else {
                intakeController.setPower(state.intakeForwardSpeed);
                stageController.setPower(0);
            }
        }
        else {
            intakeController.setPower(state.intakeForwardSpeed);
            stageController.setPower(state.feederForwardSpeedDefault);
        }
    }
    else {
        intakeController.setPower(0);
        stageController.setPower(0);
    }
}

void Feeder::resetState()
{
    state.feederState = FeederState::FEEDER_DISABLE;
    resetIntakeSensor();
}
