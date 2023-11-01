#pragma once

#include "valkyrie/BaseSubsystem.h"
#include "Constants.h"
#include "valkyrie/controllers/NeoController.h"

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Feeder : public valor::BaseSubsystem
{
public:

    Feeder(frc::TimedRobot *robot);

    ~Feeder();

    void init() override;

    void assessInputs() override;
    void analyzeDashboard() override;
    void assignOutputs() override;

    void resetState() override;

    void InitSendable(wpi::SendableBuilder& builder) override;

    enum FeederStates {
        DISABLED,
        INTAKE,
        OUTTAKE
    };
    
    struct x
    {
        FeederStates feederState;

        double intakeSpeed;
        double outtakeSpeed;
    }state;

private:
    valor::NeoController feederMotor;

};