/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//start button pulls swerve 0 positions from file
//back button pushes current swerve positions to file


#include "subsystems/Shooter.h"
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <iostream>

Shooter::Shooter() : ValorSubsystem(),
                     flywheel_lead{ShooterConstants::CAN_ID_FLYWHEEL_LEAD},
                     flywheel_follow{ShooterConstants::CAN_ID_FLYWHEEL_FOLLOW},
                     turret{ShooterConstants::CAN_ID_TURRET, rev::CANSparkMax::MotorType::kBrushless},
                     hood{ShooterConstants::CAN_ID_HOOD, rev::CANSparkMax::MotorType::kBrushless},
                     operatorController(NULL)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Shooter::setDrivetrain(Drivetrain *dt){
    odom = dt;
}

void Shooter::init()
{
    limeTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    initTable("Shooter");
    table->PutBoolean("Home Turret", false);
    table->PutNumber("Flywheel Primed Value", 1);
    table->PutNumber("Flywheel Default Value", 0.5);
    table->PutNumber("Hood Top Position", 200);
    table->PutNumber("Hood Bottom Position", 0);

    //do we have to do this?
    flywheel_follow.ConfigFactoryDefault();
    flywheel_follow.ConfigAllowableClosedloopError(0, 0);
    flywheel_follow.Config_IntegralZone(0, 0);
    flywheel_follow.Config_kF(0, ShooterConstants::flywheelKFF);
    flywheel_follow.Config_kD(0, ShooterConstants::flywheelKD);
    flywheel_follow.Config_kI(0, ShooterConstants::flywheelKI);
    flywheel_follow.Config_kP(0, ShooterConstants::flywheelKP);
    flywheel_follow.ConfigMotionAcceleration(ShooterConstants::flywheelMaxAccel);
    flywheel_follow.ConfigMotionCruiseVelocity(ShooterConstants::flywheelCruiseVelo);
    flywheel_follow.SetNeutralMode(NeutralMode::Coast);

    flywheel_lead.ConfigFactoryDefault();
    flywheel_lead.ConfigAllowableClosedloopError(0, 0);
    flywheel_lead.Config_IntegralZone(0, 0);
    flywheel_lead.Config_kF(0, ShooterConstants::flywheelKFF);
    flywheel_lead.Config_kD(0, ShooterConstants::flywheelKD);
    flywheel_lead.Config_kI(0, ShooterConstants::flywheelKI);
    flywheel_lead.Config_kP(0, ShooterConstants::flywheelKP);
    flywheel_lead.ConfigMotionAcceleration(ShooterConstants::flywheelMaxAccel);
    flywheel_lead.ConfigMotionCruiseVelocity(ShooterConstants::flywheelCruiseVelo);
    flywheel_lead.SetNeutralMode(NeutralMode::Coast);

    flywheel_lead.SetInverted(true);
    flywheel_follow.SetInverted(false);

    flywheel_follow.Follow(flywheel_lead);
    
    turret.RestoreFactoryDefaults();
    hood.RestoreFactoryDefaults();

    turret.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    hood.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    turret.SetInverted(false);

    hood.SetInverted(false);

    turret.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    turret.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, ShooterConstants::limitLeft);

    turret.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    turret.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, ShooterConstants::limitRight);

    hood.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    hood.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, ShooterConstants::limitTop);

    hood.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    hood.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, ShooterConstants::limitBottom);

    turretPidController.SetP(ShooterConstants::turretKP);
    turretPidController.SetI(ShooterConstants::turretKI);
    turretPidController.SetD(ShooterConstants::turretKD);
    turretPidController.SetIZone(ShooterConstants::turretKIZ);
    turretPidController.SetFF(ShooterConstants::turretKFF);
    turretPidController.SetOutputRange(-1, 1);

    turretPidController.SetSmartMotionMaxVelocity(ShooterConstants::turretMaxV);
    turretPidController.SetSmartMotionMinOutputVelocity(ShooterConstants::turretMinV);
    turretPidController.SetSmartMotionMaxAccel(ShooterConstants::turretMaxAccel);
    turretPidController.SetSmartMotionAllowedClosedLoopError(ShooterConstants::turretAllowedError);

    resetState();
}

void Shooter::resetState(){
    resetEncoder();
    state.turretState = TurretState::TURRET_DEFAULT;
    state.hoodState = HoodState::HOOD_DISABLE;
    state.flywheelState = FlywheelState::FLYWHEEL_DEFAULT;
    state.trackCorner = false;
}

void Shooter::resetEncoder(){
    turretEncoder.SetPosition(0);
    hoodEncoder.SetPosition(0);
}

void Shooter::assessInputs()
{
    if (!operatorController)
    {
        return;
    }

    state.leftStickX = operatorController->GetLeftX();
    state.startButton = operatorController->GetStartButton();
    state.backButton = operatorController->GetBackButton(); 
    state.rightBumper = operatorController->GetRightBumper();
    
    //Turret
    if (std::abs(state.leftStickX) > ShooterConstants::kDeadband) {
        state.turretState = TurretState::TURRET_MANUAL;
    }
    else if(state.startButton){
       state.turretState = TurretState::TURRET_PRIME;
    }
    else if(state.backButton){
        state.turretState = TurretState::TURRET_DEFAULT;
    }

    //Hood
    if(state.startButton){
        state.hoodState = HoodState::HOOD_PRIME;
    }
    else if(state.backButton){
        state.hoodState = HoodState::HOOD_DISABLE;
    }

    //Flywheel
    if(state.startButton){
        state.flywheelState = FlywheelState::FLYWHEEL_PRIME;
    }
    else if (state.backButton){
        state.flywheelState = FlywheelState::FLYWHEEL_DEFAULT;
    }

    if (state.rightBumper){
        state.trackCorner = true;
    }
    else{
        state.trackCorner = false;
    }
}

void Shooter::analyzeDashboard()
{
    if(table->GetBoolean("Home Turret", false)){
        state.turretState = TurretState::TURRET_HOME;
    }

    //slider
    state.flywheelLow = table->GetNumber("Flywheel Default Value", .5);
    state.flywheelHigh = table->GetNumber("Flywheel Primed Value", 1);

    state.hoodLow = table->GetNumber("Hood low position", 0);
    state.hoodHigh = table->GetNumber("Hood high position", 2000);
}

void Shooter::assignOutputs()
{   
    state.turretOutput = 0;
    state.turretTarget = 0;

    state.flywheelTarget = 0;
    state.hoodTarget = 0;

    bool useSmartMotion = false;

    //MANUAL
    if (state.turretState == TurretState::TURRET_MANUAL) {
        int sign = state.leftStickX >= 0 ? 1 : -1;
        state.turretOutput = sign * std::pow(state.leftStickX, 2) * ShooterConstants::TURRET_SPEED_MULTIPLIER;

        // Minimum power deadband
        if (std::abs(state.turretOutput) < ShooterConstants::pDeadband) {
            state.turretOutput = 0;
        }
        // Stop deadband
        else if (std::abs(state.turretOutput) < ShooterConstants::pSoftDeadband) {
            int direction = 1;
            if (state.turretOutput < 0) direction = -1;
            state.turretOutput = ShooterConstants::pSoftDeadband * direction;
        }
    }

    //HOME
    else if(state.turretState == TurretState::TURRET_HOME){
        state.turretTarget = ShooterConstants::homeFrontPosition;
        useSmartMotion = true;
    }

    //PRIMED
    else if (state.turretState == TurretState::TURRET_PRIME){
        float tx = limeTable->GetNumber("tx", 0.0);
        float tv = limeTable->GetNumber("tv" , 0.0);
        state.turretOutput = tv * -tx * ShooterConstants::limelightTurnKP;
    }

    //DEFAULT
    else if (state.turretState == TurretState::TURRET_DEFAULT){
        //Odometry tracking
        frc::Pose2d currentPose = odom->getPose_m();
        double targetX = ShooterConstants::hubX;
        double targetY = ShooterConstants::hubY;
        if (state.trackCorner){
            targetX = ShooterConstants::cornerX;
            targetY = ShooterConstants::cornerY;
        }

        state.turretTarget = getTargetTics(currentPose.X().to<double>(), currentPose.Y().to<double>(), currentPose.Rotation().Radians().to<double>(),
                                        targetX, targetY
                                        , ShooterConstants::ticsPerRev, ShooterConstants::falconGearRatio);
        useSmartMotion = true;
    }

    if (useSmartMotion){
        turretPidController.SetReference(state.turretTarget, rev::ControlType::kSmartMotion);
    }
    else{
        turretPidController.SetReference(state.turretOutput, rev::ControlType::kDutyCycle);
    }
    
    if(state.flywheelState == FlywheelState::FLYWHEEL_DISABLE){
        state.flywheelTarget = 0;
    }
    else if(state.flywheelState == FlywheelState::FLYWHEEL_PRIME){
        state.flywheelTarget = state.flywheelHigh;
    }
    else if (state.flywheelState == FlywheelState::FLYWHEEL_DEFAULT){
        state.flywheelTarget = state.flywheelLow;
    }
    
    double rpm = state.flywheelTarget * ShooterConstants::falconMaxRPM;
    double rp100ms = rpm / 600.0;
    double ticsp100ms = rp100ms * ShooterConstants::falconGearRatio * ShooterConstants::ticsPerRev;

    flywheel_lead.Set(ControlMode::Velocity, ticsp100ms);

    if(state.hoodState == HoodState::HOOD_DISABLE){
        state.hoodTarget = state.hoodLow;
    }
    else if(state.hoodState == HoodState::HOOD_PRIME){
        state.hoodTarget = state.hoodHigh;
    }
    hoodPidController.SetReference(state.hoodTarget, rev::ControlType::kSmartMotion);
}

double Shooter::getTargetTics(double x, 
double y,
double theta,
double hubX, 
double hubY,
double ticsPerRev,
double gearRatio){

    double deltaX = hubX - x;
    double deltaY = hubY - y;

    double targetThetaRad = atan2(deltaY, deltaX);
    double relativeAngle = targetThetaRad - theta;
    double rot = relativeAngle / (2 * M_PI);
    double targetTics = rot * ticsPerRev * gearRatio;

    while (targetTics > .5 * ticsPerRev * gearRatio){
        targetTics -= ticsPerRev * gearRatio;
    }
    while (targetTics < -.5 * ticsPerRev * gearRatio){
        targetTics += ticsPerRev * gearRatio;
    }
    return convertTargetTics(targetTics, turretEncoder.GetPosition(), ticsPerRev);
}

double Shooter::convertTargetTics(double originalTarget, double currentTics, double ticsPerRev){
    //current tics will always be between limitLeft to limitRight
    while (originalTarget < ShooterConstants::limitLeft){
        originalTarget += ticsPerRev;
    }
    while(originalTarget > ShooterConstants::limitRight){
        originalTarget -= ticsPerRev;
    }
}

