#include "subsystems/Drivetrain.h"
#include "subsystems/Shooter.h"
#include "subsystems/Feeder.h"
#include "subsystems/Turrettracker.h"
#include "auto/ValorAutoAction.h"

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>

#include <string>
#include <vector>
#include <map>

#ifndef VALOR_AUTO_H
#define VALOR_AUTO_H

class ValorAuto {
    public:
        ValorAuto(std::map<std::string, frc::Translation2d>*, Drivetrain*, Shooter*, Feeder*, TurretTracker*);
        void readPointsCSV(std::string);
        void makeAuto(std::string);
    protected:

        frc::Trajectory createTrajectory(std::vector<frc::Pose2d>& poses, bool reversed = false);
        frc2::SwerveControllerCommand<4> createTrajectoryCommand(frc::Trajectory);

        frc2::InstantCommand getSetStateFeederCommand(Feeder::FeederState);

        void readAuto(std::string);

    private:
        static frc::TrajectoryConfig config;
        std::vector<ValorAutoAction> autoActions;

        frc::ProfiledPIDController<units::radians> thetaController{
                DriveConstants::KPT,
                DriveConstants::KIT,
                DriveConstants::KDT,
                frc::ProfiledPIDController<units::radians>::Constraints(
                    units::angular_velocity::radians_per_second_t{SwerveConstants::AUTO_MAX_ROTATION_RPS},
                    units::angular_acceleration::radians_per_second_squared_t{SwerveConstants::AUTO_MAX_ROTATION_ACCEL_RPSS})
        };

        std::map<std::string, frc::Translation2d> *points;
        Drivetrain *drivetrain;
        Shooter *shooter;
        Feeder *feeder;
        TurretTracker *turretTracker;
        frc::SendableChooser<frc2::Command*> m_chooser;
};
#endif