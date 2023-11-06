#include <iostream>
#include "Intake.h"

#define DEFAULT_INTAKE_SPD 0.8f
#define DEFAULT_OUTTAKE_SPD -0.8f

Intake::Intake(frc::TimedRobot *_robot) : valor::BaseSubsystem(_robot, "Intake"),
    intakeMotor(CANIDs::INTAKE_LEAD_CAN, valor::NeutralMode::Coast, false, "")
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Intake::~Intake()
{

}

void Intake::resetState()
{
    state.intakeState = DISABLED;
}

void Intake::init()
{
    state.intakeSpeed = DEFAULT_INTAKE_SPD;
    state.outtakeSpeed = DEFAULT_OUTTAKE_SPD;

    table->PutNumber("Intake Speed", state.intakeSpeed);
    table->PutNumber("Outtake Speed", state.outtakeSpeed);

    resetState();
}

void Intake::assessInputs()
{
    if (driverGamepad->leftTriggerActive() || driverGamepad->rightTriggerActive()) {
        state.intakeState = INTAKE;
    } else if (driverGamepad->GetLeftBumper()) {
        state.intakeState = OUTTAKE;
    } else {
        state.intakeState = DISABLED;
    }
}

void Intake::analyzeDashboard()
{
    state.intakeSpeed = table->GetNumber("Intake Speed", DEFAULT_INTAKE_SPD);
    state.outtakeSpeed = table->GetNumber("Outtake Speed", DEFAULT_OUTTAKE_SPD);
}

void Intake::assignOutputs()
{
    if (state.intakeState == DISABLED) {
        intakeMotor.setPower(0);
    } else if (state.intakeState == INTAKE) {
        intakeMotor.setPower(state.intakeSpeed);
    } else if (state.intakeState == OUTTAKE) {
        intakeMotor.setPower(state.outtakeSpeed);
    }
}

void Intake::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty(
        "Intake State",
        [this]{ return state.intakeState; },
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
