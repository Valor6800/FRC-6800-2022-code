/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/TurretTracker.h"

TurretTracker::TurretTracker() : ValorSubsystem()
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void TurretTracker::init() {
    initTable("TurretTracker");
    table->PutBoolean("Use Turret Shoot", false);
    table->PutNumber("Joystick Multiplier", ShooterConstants::jMultiplier);
    table->PutNumber("Delta Heading", 0);
}

void TurretTracker::setDrivetrain(Drivetrain *dt){
    drivetrain = dt;
}

void TurretTracker::setShooter(Shooter *sh){
    shooter = sh;
}

void TurretTracker::assessInputs() {
    
}

void TurretTracker::analyzeDashboard() {
    state.jMultiplier = table->GetNumber("Joystick Multiplier", ShooterConstants::jMultiplier);
}

double TurretTracker::tMinusJ(double robotHeading, double turretPos, double jx, double jy)
{
    double turretHeading = robotHeading - 90 + turretPos;
    if (turretHeading < -180) turretHeading += 360;
    if (turretHeading > 180) turretHeading -= 360;

    double tx = (shooter->state.distanceToHub) * cos(turretHeading * MathConstants::toRadians);
    double ty = (shooter->state.distanceToHub) * sin(turretHeading * MathConstants::toRadians);

    jx *= state.jMultiplier;
    jy *= state.jMultiplier;

    double rx = tx - jx;
    double ry = ty - jy;

    double turretHeadingDesired = atan2(ry, rx);
    double deltaHeading = turretHeading - (turretHeadingDesired * MathConstants::toDegrees);

    if (deltaHeading < -180) deltaHeading += 360;
    if (deltaHeading > 180) deltaHeading -= 360;
    
    table->PutNumber("Delta Heading", deltaHeading);
    return -deltaHeading;
}

void TurretTracker::assignOutputs() {
    double tv = shooter->state.tv;

    double robotHeading = drivetrain->getPose_m().Rotation().Degrees().to<double>();
    double turretPos = shooter->turretEncoder.GetPosition();
    double jx = drivetrain->state.leftStickX;
    double jy = -1 * (drivetrain->state.leftStickY);

    if (tv == 1) {
        state.cachedTx = shooter->state.tx;
        // 0.75 = limeligh KP
        state.target = (-state.cachedTx * 0.75) + shooter->turretEncoder.GetPosition();

        state.target += tMinusJ(robotHeading, turretPos, jx, jy);

        state.cachedHeading = robotHeading;
        state.cachedX = drivetrain->getPose_m().X().to<double>();
        state.cachedY = drivetrain->getPose_m().Y().to<double>();
        state.cachedTurretPos = turretPos;
    }
    else {
        if (table->GetBoolean("Use Turret Shoot", false))
            state.target = -1 * robotHeading + state.cachedTurretPos - state.cachedTx;
        else
            state.target = turretPos;
    }

    if (state.target < -90) {
        state.target += 360;
    }
    else if (state.target > 270) {
        state.target -= 360;
    }

    if (state.target < 0) {
        state.target = 0;
    }
    else if (state.target > 180) {
        state.target = 180;
    }

    shooter->assignTurret(state.target);
}