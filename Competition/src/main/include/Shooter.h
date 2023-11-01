#pragma once

#include "valkyrie/BaseSubsystem.h"
#include "Constants.h"
#include "valkyrie/controllers/FalconController.h"

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Shooter : public valor::BaseSubsystem
{
public:

    Shooter(frc::TimedRobot *robot);

    ~Shooter();

    void init() override;

    void assessInputs() override;
    void analyzeDashboard() override;
    void assignOutputs() override;

    void resetState() override;

    void InitSendable(wpi::SendableBuilder& builder) override;

    enum ShooterState {
        IDLE,
        SHOOT
    };
    
    struct x
    {
        ShooterState shooterState;

        double idleSpeed;
        double shootSpeed;
    }state;

private:
    valor::FalconController shooterMotor;

};