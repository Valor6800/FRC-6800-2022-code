#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>

#include "Constants.h"
#include "valkyrie/Gamepad.h"

#include "Drivetrain.h"
#include "Elevarm.h"
#include "Intake.h"
#include "Leds.h"

#include <frc/DriverStation.h>
#include <frc/DataLogManager.h>

#include <frc/livewindow/LiveWindow.h>

#include <fstream>

class Robot : public frc::TimedRobot {
    public:
        Robot();

        void RobotInit() override;
        void RobotPeriodic() override;
        void DisabledInit() override;
        void DisabledPeriodic() override;
        void AutonomousInit() override;
        void AutonomousPeriodic() override;
        void TeleopInit() override;
        void TeleopPeriodic() override;
        void TestPeriodic() override;
        void AutonomousExit() override;
        
    private:
        valor::Gamepad gamepadOperator{OIConstants::GAMEPAD_OPERATOR_LOCATION};
        valor::Gamepad gamepadDriver{OIConstants::GAMEPAD_BASE_LOCATION};

        frc2::Command * autoCommand = nullptr;

        Drivetrain drivetrain;
        Intake intake;
        Elevarm elevarm;
        Leds leds;

        std::ofstream outfile;
};
