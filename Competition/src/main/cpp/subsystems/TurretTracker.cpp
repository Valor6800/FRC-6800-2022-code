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
    limeTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
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

void TurretTracker::assignOutputs() {
    // state.cachedVX = drivetrain->getKinematics().ToChassisSpeeds().vx.to<double>();
    // state.cachedVY = drivetrain->getKinematics().ToChassisSpeeds().vy.to<double>();
    // state.cachedVT = drivetrain->getKinematics().ToChassisSpeeds().omega.to<double>();

    double tv = limeTable->GetNumber("tv", 0.0);

    if (tv == 1) {
        state.cachedTx = limeTable->GetNumber("tx", 0.0);
        // 0.75 = limeligh KP
        state.target = (-state.cachedTx * 0.75) + shooter->turretEncoder.GetPosition();
        
        state.cachedHeading = drivetrain->getPose_m().Rotation().Degrees().to<double>();
        state.cachedX = drivetrain->getPose_m().X().to<double>();
        state.cachedY = drivetrain->getPose_m().Y().to<double>();
        state.cachedTurretPos = shooter->turretEncoder.GetPosition();
    }
    else {
        double changeHeading = drivetrain->getPose_m().Rotation().Degrees().to<double>() - state.cachedHeading;
        if (changeHeading > 180){
            changeHeading -= 180;
        }
        if (changeHeading < -180){
            changeHeading += 180;
        }

        state.target -= (changeHeading);
        //state.target = shooter->turretEncoder.GetPosition();
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