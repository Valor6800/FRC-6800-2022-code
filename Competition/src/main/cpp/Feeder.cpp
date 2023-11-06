#include <iostream>
#include "Feeder.h"

#define DEFAULT_INTAKE_SPD 0.8f
#define DEFAULT_OUTTAKE_SPD -0.8f

Feeder::Feeder(frc::TimedRobot *_robot) : valor::BaseSubsystem(_robot, "Feeder"),
    feederMotor(CANIDs::FEEDER_CAN, valor::NeutralMode::Coast, true, "")
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Feeder::~Feeder()
{

}

void Feeder::resetState()
{
    state.feederState = DISABLED;
}

void Feeder::init()
{
    state.intakeSpeed = DEFAULT_INTAKE_SPD;
    state.outtakeSpeed = DEFAULT_OUTTAKE_SPD;

    table->PutNumber("Intake Speed", state.intakeSpeed);
    table->PutNumber("Outtake Speed", state.outtakeSpeed);

    resetState();
}

void Feeder::assessInputs()
{
    if (driverGamepad->rightTriggerActive()) {
        state.feederState = INTAKE;
    } else if (driverGamepad->GetLeftBumper()) {
        state.feederState = OUTTAKE;
    } else {
        state.feederState = DISABLED;
    }
}

void Feeder::analyzeDashboard()
{
    state.intakeSpeed = table->GetNumber("Intake Speed", DEFAULT_INTAKE_SPD);
    state.outtakeSpeed = table->GetNumber("Outtake Speed", DEFAULT_OUTTAKE_SPD);
}

void Feeder::assignOutputs()
{
    if (state.feederState == DISABLED) {
        feederMotor.setPower(0);
    } else if (state.feederState == INTAKE) {
        feederMotor.setPower(state.intakeSpeed);
    } else if (state.feederState == OUTTAKE) {
        feederMotor.setPower(state.outtakeSpeed);
    }
}

void Feeder::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty(
        "Feeder State",
        [this]{ return state.feederState; },
        nullptr
    );
    builder.AddDoubleProperty(
        "Intake Speed",
        [this]{ return state.intakeSpeed; },
        nullptr
    );
    builder.AddDoubleProperty(
        "Outtake Speed",
        [this]{ return state.outtakeSpeed; },
        nullptr
    );
}
