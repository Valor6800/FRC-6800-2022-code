/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"



#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {
    //m_container.m_drivetrain.resetState();
    // m_container.m_drivetrain.setKF();
    // m_container.m_drivetrain.pullSwerveModuleZeroReference();
    m_container.m_shooter.resetState();
    m_container.m_auto.fillAutoList();
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
    m_container.m_feeder.robotMode = ValorSubsystem::RobotMode::DISABLED;
    m_container.m_drivetrain.robotMode = ValorSubsystem::RobotMode::DISABLED;
    m_container.m_shooter.robotMode = ValorSubsystem::RobotMode::DISABLED;
    m_container.m_turretTracker.robotMode = ValorSubsystem::RobotMode::DISABLED;
    m_container.m_lift.robotMode = ValorSubsystem::RobotMode::DISABLED; //just added, not tested

    m_container.m_shooter.resetState();
    m_container.m_drivetrain.resetState();
    m_container.m_lift.resetState();
    m_container.m_turretTracker.resetState();
    //m_container.m_feeder.resetState(); //just added, not tested

    m_container.m_drivetrain.setMotorMode(false);

    //m_container.m_drivetrain.pullSwerveModuleZeroReference();
}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
    m_container.m_drivetrain.setMotorMode(true);

    //might need to add back
    m_container.m_shooter.resetState();

    m_autonomousCommand = m_container.GetAutonomousCommand();
    if (m_autonomousCommand != nullptr) {
        m_autonomousCommand->Schedule();
    }

    m_container.m_feeder.robotMode = ValorSubsystem::RobotMode::AUTO;
    m_container.m_drivetrain.robotMode = ValorSubsystem::RobotMode::AUTO;
    m_container.m_shooter.robotMode = ValorSubsystem::RobotMode::AUTO;
    m_container.m_lift.robotMode = ValorSubsystem::RobotMode::AUTO;
    m_container.m_turretTracker.robotMode = ValorSubsystem::RobotMode::AUTO;
    m_container.m_turretTracker.disableWrapAround();

    m_container.m_drivetrain.pullSwerveModuleZeroReference();
}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {
    m_container.m_drivetrain.setMotorMode(false);


    if (m_autonomousCommand != nullptr) {
        m_autonomousCommand->Cancel();
        m_autonomousCommand = nullptr;
    }

    m_container.m_shooter.setLimelight(0);

    m_container.m_feeder.robotMode = ValorSubsystem::RobotMode::TELEOP;
    m_container.m_drivetrain.robotMode = ValorSubsystem::RobotMode::TELEOP;
    m_container.m_shooter.robotMode = ValorSubsystem::RobotMode::TELEOP;
    m_container.m_shooter.state.turretState = m_container.m_shooter.TURRET_TRACK;
    m_container.m_shooter.state.hoodState = m_container.m_shooter.HOOD_TRACK;
    m_container.m_shooter.state.flywheelState = m_container.m_shooter.FLYWHEEL_TRACK;

    m_container.m_lift.robotMode = ValorSubsystem::RobotMode::TELEOP;
    m_container.m_turretTracker.robotMode = ValorSubsystem::RobotMode::TELEOP; 
    m_container.m_turretTracker.enableWrapAround();

}



/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
