#pragma once
#include <functional>
#include <frc/TimedRobot.h>

#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/sensors/CANdleSensor.h"
#include "Constants.h"

#include "Elevarm.h"
#include "Drivetrain.h"
#include "Intake.h"

#include "ctre/Phoenix.h"
#include "ctre/phoenix/led/CANdle.h"
#include "ctre/phoenix/led/StrobeAnimation.h"

#include <vector>
class Leds : public valor::BaseSubsystem
{
    public:
        Leds(frc::TimedRobot *robot, Elevarm *elevarm, Intake *intake, Drivetrain *drivetrain);
        valor::CANdleSensor::RGBColor value;
        
        void init();
        void assessInputs();
        void analyzeDashboard();
        void assignOutputs();
        void resetState();
        void InitSendable(wpi::SendableBuilder& builder) override;

        struct State{
            std::vector<units::second_t> startedAnimating; 
        } state;
        
    private:
        Elevarm *elevarm;
        Intake *intake;
        Drivetrain *drivetrain;
        valor::CANdleSensor candle;

};