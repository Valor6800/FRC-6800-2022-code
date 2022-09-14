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
                           motor_rotateMain(FeederConstants::MOTOR_ROTATE_MAIN_CAN_ID, "baseCAN"),
                           motor_rotateFollow(FeederConstants::MOTOR_ROTATE_FOLLOW_CAN_ID, "baseCAN"),

                           banner(FeederConstants::BANNER_DIO_PORT)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Feeder::init()
{
    initTable("Feeder");
    motor_intake.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
    motor_intake.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    motor_intake.SetInverted(false);
    motor_intake.EnableVoltageCompensation(false);
    motor_intake.ConfigVoltageCompSaturation(10);
    motor_intake.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 60, 80, .75)); //potentially could do 40 60

    motor_stage.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
    motor_stage.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    motor_stage.SetInverted(true);
    motor_stage.EnableVoltageCompensation(true);
    motor_stage.ConfigVoltageCompSaturation(10);

    

    
    motor_rotateMain.ConfigForwardSoftLimitThreshold(FeederConstants::rotateForwardLimit);
    motor_rotateMain.ConfigReverseSoftLimitThreshold(FeederConstants::rotateReverseLimit);

    motor_rotateMain.ConfigForwardSoftLimitEnable(true);
    motor_rotateMain.ConfigReverseSoftLimitEnable(true);

    motor_rotateMain.SetSelectedSensorPosition(0);
    motor_rotateMain.SetInverted(false); // needs to be tested
    motor_rotateFollow.SetInverted(true); // needs to be tested
    motor_rotateFollow.Follow(motor_rotateMain);   

    motor_rotateMain.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    motor_rotateFollow.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);

    motor_rotateMain.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
    motor_rotateMain.ConfigAllowableClosedloopError(0, 0);
    motor_rotateMain.Config_IntegralZone(0, 0);

    motor_rotateMain.Config_kF(0, FeederConstants::main_KF);
    motor_rotateMain.Config_kD(0, FeederConstants::main_KD);
    motor_rotateMain.Config_kI(0, FeederConstants::main_KI);
    motor_rotateMain.Config_kP(0, FeederConstants::main_KP);
    motor_rotateMain.ConfigMotionAcceleration(FeederConstants::MAIN_MOTION_ACCELERATION);
    motor_rotateMain.ConfigMotionCruiseVelocity(FeederConstants::MAIN_MOTION_CRUISE_VELOCITY);

    table->PutBoolean("Reverse Feeder?", false);
    table->PutNumber("Intake Reverse Speed", FeederConstants::DEFAULT_INTAKE_SPEED_REVERSE);
    table->PutNumber("Feeder Reverse Speed", FeederConstants::DEFAULT_FEEDER_SPEED_REVERSE);
    table->PutNumber("Intake Forward Speed", FeederConstants::DEFAULT_INTAKE_SPEED_FORWARD);
    table->PutNumber("Feeder Forward Speed Default", FeederConstants::DEFAULT_FEEDER_SPEED_FORWARD_DEFAULT);
    table->PutNumber("Feeder Forward Speed Shoot", FeederConstants::DEFAULT_FEEDER_SPEED_FORWARD_SHOOT);
    table->PutNumber("Intake Spike Current", FeederConstants::JAM_CURRENT);

    table->PutNumber("Average Amps", 0);
    table->PutBoolean("Spiked: ", 0);
    table->PutBoolean("Banner: ", 0);
    
    resetState();
    currentSensor.setGetter([this]() { return motor_intake.GetOutputCurrent(); });
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
    else if (driverController->GetBButton()) {
        state.feederState = FeederState::FEEDER_RETRACT; //Set Intake rotate  to upper setpoint
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
    state.intakeReverseSpeed = table->GetNumber("Intake Reverse Speed", FeederConstants::DEFAULT_INTAKE_SPEED_REVERSE);
    state.feederReverseSpeed = table->GetNumber("Feeder Reverse Speed", FeederConstants::DEFAULT_FEEDER_SPEED_REVERSE);
    state.intakeForwardSpeed = table->GetNumber("Intake Forward Speed", FeederConstants::DEFAULT_INTAKE_SPEED_FORWARD);
    state.feederForwardSpeedDefault = table->GetNumber("Feeder Forward Speed Default", FeederConstants::DEFAULT_FEEDER_SPEED_FORWARD_DEFAULT);
    state.feederForwardSpeedShoot = table->GetNumber("Feeder Forward Speed Shoot", FeederConstants::DEFAULT_FEEDER_SPEED_FORWARD_SHOOT);

    table->PutNumber("Average Amps", currentSensor.getSensor());
    table->PutBoolean("Spiked: ", currentSensor.spiked());
    table->PutBoolean("Banner: ", debounceSensor.getSensor());
    table->PutNumber("current feeder state", state.feederState);

}

void Feeder::assignOutputs()
{

    if (state.feederState == FeederState::FEEDER_DISABLE) {
        motor_intake.Set(0);
        motor_stage.Set(0);
        motor_rotateMain.Set(ControlMode::MotionMagic, FeederConstants::rotateForwardLimit);
    }
    else if (state.feederState == FeederState::FEEDER_SHOOT) {
        motor_intake.Set(state.intakeForwardSpeed);
        motor_stage.Set(state.feederForwardSpeedShoot);
    }
    else if (state.feederState == FeederState::FEEDER_RETRACT){
        motor_rotateMain.Set(ControlMode::MotionMagic, FeederConstants::rotateReverseLimit); // set rotation to be up
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
        if (debounceSensor.getSensor()) {
            if (debounceSensor.spiked()) {
                resetIntakeSensor();
            }
            if (currentSensor.spiked()) {
                motor_intake.Set(0);
                motor_stage.Set(0);
            }
            else {
                motor_intake.Set(state.intakeForwardSpeed);
                motor_stage.Set(0);
            }
        }
        else {
            motor_intake.Set(state.intakeForwardSpeed);
            motor_stage.Set(state.feederForwardSpeedDefault);
        }
    }
    else {
        motor_intake.Set(0);
        motor_stage.Set(0);
    }
}

void Feeder::resetState()
{
    state.feederState = FeederState::FEEDER_DISABLE;
    resetIntakeSensor();
}
