#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

#include <ctime>

Robot::Robot() : drivetrain(this), intake(this), feeder(this), shooter(this)
{
    frc::TimedRobot();
}

void Robot::RobotInit() {
    drivetrain.setGamepads(&gamepadOperator, &gamepadDriver);
    drivetrain.resetState();

    intake.setGamepads(&gamepadOperator, &gamepadDriver);
    intake.resetState();

    feeder.setGamepads(&gamepadOperator, &gamepadDriver);
    feeder.resetState();

    shooter.setGamepads(&gamepadOperator, &gamepadDriver);
    shooter.resetState();

    frc::LiveWindow::EnableAllTelemetry();
    frc::DataLogManager::Start();
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
void Robot::DisabledInit() { }

void Robot::DisabledPeriodic() { }

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
    drivetrain.resetState();
    drivetrain.state.matchStart = frc::Timer::GetFPGATimestamp().to<double>();
    drivetrain.setLimelightPipeline(Drivetrain::LimelightPipes::APRIL_TAGS);
    drivetrain.setDriveMotorNeutralMode(valor::NeutralMode::Brake);
    drivetrain.pullSwerveModuleZeroReference();
}

void Robot::AutonomousExit() {
    drivetrain.state.xPose = true;

}

void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {
    // drivetrain.pullSwerveModuleZeroReference();
    drivetrain.setDriveMotorNeutralMode(valor::NeutralMode::Coast);

    if (autoCommand != nullptr) {
        autoCommand->Cancel();
        autoCommand = nullptr;
    }
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
