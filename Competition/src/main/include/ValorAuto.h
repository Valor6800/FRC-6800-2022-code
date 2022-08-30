#include "subsystems/Drivetrain.h"
#include "subsystems/Feeder.h"
#include "auto/ValorPoints.h"

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>

#ifndef VALOR_AUTO_H
#define VALOR_AUTO_H

class ValorAuto {
    public:
        ValorAuto(ValorPoints*, Drivetrain*, Feeder*);
    protected:

        frc::Trajectory createTrajectory(std::vector<frc::Pose2d>& poses, bool reversed = false);
        frc2::SwerveControllerCommand<4> createTrajectoryCommand(frc::Trajectory);

        frc2::InstantCommand getSetStateFeederCommand(Feeder::FeederState);

        void test();

    private:
        static frc::TrajectoryConfig config;

        frc::ProfiledPIDController<units::radians> thetaController{
                DriveConstants::KPT,
                DriveConstants::KIT,
                DriveConstants::KDT,
                frc::ProfiledPIDController<units::radians>::Constraints(
                    units::angular_velocity::radians_per_second_t{SwerveConstants::AUTO_MAX_ROTATION_RPS},
                    units::angular_acceleration::radians_per_second_squared_t{SwerveConstants::AUTO_MAX_ROTATION_ACCEL_RPSS})
        };

        ValorPoints *points;
        Drivetrain *drivetrain;
        Feeder *feeder;
};

#endif