#include <iostream>
#include "Shooter.h"

#define DEFAULT_IDLE_SPD 0.2f
#define DEFAULT_SHOOT_SPD 0.4f

Shooter::Shooter(frc::TimedRobot *_robot) : valor::BaseSubsystem(_robot, "Shooter"),
    shooterMotor(CANIDs::SHOOTER_CAN, valor::NeutralMode::Coast, false, "baseCAN")
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Shooter::~Shooter()
{

}

void Shooter::resetState()
{
    state.shooterState = IDLE;
}

void Shooter::init()
{
    state.idleSpeed = DEFAULT_IDLE_SPD;
    state.shootSpeed = DEFAULT_SHOOT_SPD;

    table->PutNumber("Idle Speed", state.idleSpeed);
    table->PutNumber("Shoot Speed", state.shootSpeed);

    resetState();
}

void Shooter::assessInputs()
{
    if (driverGamepad->GetYButtonPressed()) {
        state.shooterState = SHOOT;
    } else if (driverGamepad->GetAButtonPressed()) {
        state.shooterState = IDLE;
    }
}

void Shooter::analyzeDashboard()
{
    state.idleSpeed = table->GetNumber("Idle Speed", DEFAULT_IDLE_SPD);
    state.shootSpeed = table->GetNumber("Shoot Speed", DEFAULT_SHOOT_SPD);
}

void Shooter::assignOutputs()
{
    if (state.shooterState == IDLE) {
        shooterMotor.setPower(state.shootSpeed);
    } else if (state.shooterState == SHOOT) {
        shooterMotor.setPower(state.idleSpeed);
    }
}

void Shooter::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty(
        "Shooter State",
        [this]{ return state.shooterState; },
        nullptr
    );
    builder.AddDoubleProperty(
        "Idle Speed",
        [this]{ return state.idleSpeed; },
        nullptr
    );
    builder.AddDoubleProperty(
        "Shoot Speed",
        [this]{ return state.shootSpeed; },
        nullptr
    );
}
