#pragma once

#include "valkyrie/BaseSubsystem.h"
#include "Constants.h"
#include "valkyrie/controllers/NeoController.h"

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Intake : public valor::BaseSubsystem
{
public:

    Intake(frc::TimedRobot *robot);

    ~Intake();

    void init() override;

    void assessInputs() override;
    void analyzeDashboard() override;
    void assignOutputs() override;

    void resetState() override;

    void InitSendable(wpi::SendableBuilder& builder) override;

    enum IntakeStates {
        DISABLED,
        INTAKE,
        OUTTAKE
    };
    
    struct x
    {
        IntakeStates intakeState;

        double intakeSpeed;
        double outtakeSpeed;
    }state;

private:
    valor::NeoController intakeMotor;

};