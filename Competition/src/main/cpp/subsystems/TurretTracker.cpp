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
    table->PutBoolean("Use Turret Shoot", true);
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

}

void TurretTracker::disableWrapAround(){
    table->PutBoolean("Use Turret Shoot", false);
}

void TurretTracker::enableWrapAround(){
    table->PutBoolean("Use Turret Shoot", true);
}



void TurretTracker::assignOutputs() {

    double tv = shooter->state.tv;
    double turretPos = shooter->turretEncoder.GetPosition();
    double robotHeading = drivetrain->getPose_m().Rotation().Degrees().to<double>();
    double x = drivetrain->getPose_m().X().to<double>();
    double y = drivetrain->getPose_m().Y().to<double>();
    double tx = shooter->state.tx;

    if (tv == 1) {
        // 0.75 = limeligh KP
        state.target = (-state.cachedTx * 0.75) + turretPos;

        state.cachedHeading = robotHeading;
        state.cachedX = x;
        state.cachedY = y;
        state.cachedTx = tx;
        state.cachedTurretPos = turretPos;
        
        state.destinationTurretHeading = robotHeading + turretPos - 90 - state.cachedTx;
    }
    else {
        if (table->GetBoolean("Use Turret Shoot", true))
            state.target = state.destinationTurretHeading - robotHeading + 90 + tx;
        else
            state.target = turretPos;
    }

    // Super Poop
    if (shooter->driverController->leftTriggerActive()) {
        double wrappedExistingHeading = state.destinationTurretHeading;

        // Wrap to positive numbers
        if (wrappedExistingHeading < 0)
            wrappedExistingHeading += 360;

        // Case structure for robot locations on the field
        double superPoopHeading = 90;
        if (wrappedExistingHeading <= 45)
            superPoopHeading += 2 * 90.0 / 4;
        else if (wrappedExistingHeading > 45 && wrappedExistingHeading <= 135)
            superPoopHeading += 1 * 90.0 / 4;
        else if (wrappedExistingHeading > 135 && wrappedExistingHeading <= 225)
            superPoopHeading += 3 * 90.0 / 4;
        else
            superPoopHeading += 90;
        
        // Convert heading to turret angle
        state.target = superPoopHeading - robotHeading + 90 + state.cachedTx;
    }

    if (state.target < -90) {
        state.target += 360;
    }
    else if (state.target > 270) {
        state.target -= 360;
    }

    if (state.target < -7) {
        state.target = -7;
    }
    else if (state.target > 190.5) {
        state.target = 190.5;
    }

    shooter->assignTurret(state.target);
}