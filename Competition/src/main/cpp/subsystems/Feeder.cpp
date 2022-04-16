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

Feeder::Feeder() : ValorSubsystem(),
                           driverController(NULL),
                           operatorController(NULL),
                           motor_intake(FeederConstants::MOTOR_INTAKE_CAN_ID, "baseCAN"),
                           motor_stage(FeederConstants::MOTOR_STAGE_CAN_ID, "baseCAN"),
                           banner(FeederConstants::BANNER_DIO_PORT),
                           colorSensor(FeederConstants::COLOR_SENSOR_DIO_PORT),
                           revColorSensor(frc::I2C::kOnboard)

{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Feeder::init()
{
    initTable("Feeder");
    limeTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

    motor_intake.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
    motor_intake.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    motor_intake.SetInverted(false);
    motor_intake.EnableVoltageCompensation(false);
    motor_intake.ConfigVoltageCompSaturation(10);
    motor_intake.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 60, 80, .75));

    motor_stage.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
    motor_stage.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    motor_stage.SetInverted(true);
    motor_stage.EnableVoltageCompensation(true);
    motor_stage.ConfigVoltageCompSaturation(10);

    table->PutBoolean("Reverse Feeder?", false);
    table->PutNumber("Intake Reverse Speed", FeederConstants::DEFAULT_INTAKE_SPEED_REVERSE);
    table->PutNumber("Feeder Reverse Speed", FeederConstants::DEFAULT_FEEDER_SPEED_REVERSE);
    table->PutNumber("Intake Forward Speed", FeederConstants::DEFAULT_INTAKE_SPEED_FORWARD);
    table->PutNumber("Feeder Forward Speed Default", FeederConstants::DEFAULT_FEEDER_SPEED_FORWARD_DEFAULT);
    table->PutNumber("Feeder Forward Speed Shoot", FeederConstants::DEFAULT_FEEDER_SPEED_FORWARD_SHOOT);
    table->PutNumber("Intake Spike Current", FeederConstants::JAM_CURRENT);

    table->PutNumber("Blue Threshold", FeederConstants::BLUE_THRESHOLD);
    table->PutNumber("Red Threshold", FeederConstants::RED_THRESHOLD);

    table->PutNumber("Average Amps", 0);
    table->PutBoolean("Spiked: ", 0);
    table->PutBoolean("Banner: ", 0);

    table->PutBoolean("Auto Poop Enabled", false);

    state.timing = false;
    ballTimer.Reset();

    fmsTable = nt::NetworkTableInstance::GetDefault().GetTable("FMSInfo");
    resetState();
}

void Feeder::setControllers(frc::XboxController *controllerO, frc::XboxController *controllerD)
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

    // driver inputs
  
    state.driver_leftBumperPressed = driverController->GetLeftBumper();
    state.driver_rightBumperPressed = driverController->GetRightBumper();

    state.driver_rightTriggerPressed = driverController->GetRightTriggerAxis() > OIConstants::kDeadBandTrigger;
    state.driver_leftTriggerPressed = driverController->GetLeftTriggerAxis() > OIConstants::kDeadBandTrigger;


    // operator inputs

    state.operator_leftBumperPressed = operatorController->GetLeftBumper();
        
    if (state.driver_leftBumperPressed) {
        state.feederState = FeederState::FEEDER_REVERSE;
        state.spiked = false;
    }
    else if(isOppositeColor() && state.autoPoopEnabled){
        if (!state.timing) {
            state.timing = true;
            ballTimer.Reset();
            ballTimer.Start();
        }
        if (ballTimer.Get() > 0.15_s && limeTable->GetNumber("tv", 0.0)) {
            state.feederState = FeederState::FEEDER_FEEDER_ONLY; //intake and feeder run
            state.spiked = false;
            ballTimer.Stop();
        }
    }
    else if (state.driver_rightTriggerPressed || state.operator_leftBumperPressed) {
        state.feederState = FeederState::FEEDER_SHOOT; //intake and feeder run
        state.spiked = false;
    }
    else if (state.driver_rightBumperPressed) {
        state.feederState = FeederState::FEEDER_REGULAR_INTAKE; //standard intake
    }
    else if (state.driver_leftTriggerPressed) {
        state.feederState = FeederState::FEEDER_CURRENT_INTAKE; //includes current/banner sensing
    }
    else{
        state.feederState = FeederState::FEEDER_DISABLE;
    }
}

void Feeder::analyzeDashboard()
{
    state.reversed = table->GetBoolean("Reverse Feeder?", false);
    state.intakeReverseSpeed = table->GetNumber("Intake Reverse Speed", FeederConstants::DEFAULT_INTAKE_SPEED_REVERSE);
    state.feederReverseSpeed = table->GetNumber("Feeder Reverse Speed", FeederConstants::DEFAULT_FEEDER_SPEED_REVERSE);
    state.intakeForwardSpeed = table->GetNumber("Intake Forward Speed", FeederConstants::DEFAULT_INTAKE_SPEED_FORWARD);
    state.feederForwardSpeedDefault = table->GetNumber("Feeder Forward Speed Default", FeederConstants::DEFAULT_FEEDER_SPEED_FORWARD_DEFAULT);
    state.feederForwardSpeedShoot = table->GetNumber("Feeder Forward Speed Shoot", FeederConstants::DEFAULT_FEEDER_SPEED_FORWARD_SHOOT);
    state.spikeCurrent = table->GetNumber("Intake Spike Current", FeederConstants::JAM_CURRENT);

    table->PutNumber("Average Amps", state.instCurrent);
    table->PutBoolean("Spiked: ", state.spiked);
    table->PutBoolean("Banner: ", state.bannerTripped);
    table->PutBoolean("Color is red: ", state.colorIsRed);

    table->PutNumber("red", revColorSensor.GetColor().red);
    table->PutNumber("blue", revColorSensor.GetColor().blue);
    table->PutNumber("green", revColorSensor.GetColor().green);
    table->PutNumber("infared", revColorSensor.GetIR());

    state.blueThreshold = table->GetNumber("Blue Threshold", FeederConstants::BLUE_THRESHOLD);
    state.redThreshold = table->GetNumber("Red Threshold", FeederConstants::RED_THRESHOLD);

    table->PutBoolean("isRed", isRed());
    table->PutBoolean("isBlue", isBlue());

    table->PutNumber("current feeder state", state.feederState);
    state.autoPoopEnabled = table->GetBoolean("Auto Poop Enabled", false);
    // Calculate instantaneous current
    calcCurrent();
}

void Feeder::assignOutputs()
{
    state.bannerTripped = !banner.Get();
    state.colorIsRed = colorSensor.Get();
    state.currentBanner = state.bannerTripped;

    if (state.feederState == FeederState::FEEDER_DISABLE) {
        motor_intake.Set(0);
        motor_stage.Set(0);
    }
    else if (state.feederState == FeederState::FEEDER_SHOOT) {
        motor_intake.Set(state.intakeForwardSpeed);
        motor_stage.Set(state.feederForwardSpeedShoot);
    }
    else if (state.feederState == Feeder::FEEDER_REVERSE) {
        motor_intake.Set(state.intakeReverseSpeed);
        motor_stage.Set(state.feederReverseSpeed);
    }
    else if (state.feederState == Feeder::FEEDER_REGULAR_INTAKE){
        motor_intake.Set(state.intakeForwardSpeed);
        motor_stage.Set(0);
    }
    else if (state.feederState == FeederState::FEEDER_CURRENT_INTAKE) { //includes currrent sensing
        if (state.bannerTripped) {
            if (state.currentBanner && !state.previousBanner) {
                resetDeque();
                state.spiked = false;
            }
            if (state.spiked) {
                motor_intake.Set(0);
                motor_stage.Set(0);
            }
            else {
                if (state.instCurrent > state.spikeCurrent && state.bannerTripped) {
                    motor_intake.Set(0);
                    motor_stage.Set(0);
                   state.spiked = true;
                }
                else {
                    motor_intake.Set(state.intakeForwardSpeed);
                    motor_stage.Set(0);
                }
            }
        }
        else {
            motor_intake.Set(state.intakeForwardSpeed);
            motor_stage.Set(state.feederForwardSpeedDefault);
        }
    }
    else if (state.feederState == FeederState::FEEDER_FEEDER_ONLY){
        motor_intake.Set(0);
        motor_stage.Set(state.feederForwardSpeedShoot);
    }
    else {
        motor_intake.Set(0);
        motor_stage.Set(0);
    }
    state.previousBanner = state.currentBanner;
}

void Feeder::calcCurrent() {
    state.current_cache.pop_front();
    state.current_cache.push_back(motor_intake.GetOutputCurrent());

    // Calculate average current over the cache size, or circular buffer window
    double sum = 0;
    for (int i = 0; i < FeederConstants::CACHE_SIZE; i++) {
        sum += state.current_cache.at(i);
    }
    state.instCurrent = sum / FeederConstants::CACHE_SIZE;
}

void Feeder::resetDeque() {
    state.current_cache.clear();
    for (int i = 0; i < FeederConstants::CACHE_SIZE; i++) {
        state.current_cache.push_back(0);
    }
    state.spiked = false;
}

void Feeder::resetState()
{
    state.feederState = FeederState::FEEDER_DISABLE;

    state.spiked = false;
    state.previousBanner = false;

    resetDeque();
}

bool Feeder::isRed(){
    return revColorSensor.GetColor().red > state.redThreshold;
}

bool Feeder::isBlue(){
    return revColorSensor.GetColor().blue > state.blueThreshold;
}

bool Feeder::isOppositeColor(){
    if (isRed() && !fmsTable->GetBoolean("IsRedAlliance", false)){
        return true;
    }
    if (isBlue() && fmsTable->GetBoolean("IsRedAlliance", false)){
        return true;
    }
    return false;
}
