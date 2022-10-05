#include "auto/ValorAuto.h"

#include "Constants.h"
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>

frc::TrajectoryConfig ValorAuto::config(
    units::velocity::meters_per_second_t{SwerveConstants::AUTO_MAX_SPEED_MPS},
    units::acceleration::meters_per_second_squared_t{SwerveConstants::AUTO_MAX_ACCEL_MPSS});

ValorAuto::ValorAuto(ValorPoints *_points, Drivetrain *_drivetrain, Feeder *_feeder) :
    points(_points),
    drivetrain(_drivetrain),
    feeder(_feeder)
{
    ValorAuto::config.SetKinematics(drivetrain->getKinematics());

    // @TODO look at angle wrapping and modding
    thetaController.EnableContinuousInput(units::radian_t(-wpi::numbers::pi),
                                          units::radian_t(wpi::numbers::pi));
}

frc2::InstantCommand ValorAuto::getSetStateFeederCommand(Feeder::FeederState feederState)
{
    return frc2::InstantCommand( [&] { feeder->state.feederState = feederState; }, {feeder} );
}

frc2::SwerveControllerCommand<4> ValorAuto::createTrajectoryCommand(frc::Trajectory trajectory)
{
    return frc2::SwerveControllerCommand<4>(
        trajectory,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );
}

frc::Trajectory ValorAuto::createTrajectory(std::vector<frc::Pose2d>& poses, bool reversed)
{
    ValorAuto::config.SetReversed(reversed);
    return frc::TrajectoryGenerator::GenerateTrajectory(poses, config);
}

void ValorAuto::readAuto(std::string fileName)
{ 
    std::ifstream infile("/home/lvuser/autos/" + fileName + ".txt");
    if (!infile.good())
        return;

    std::string line;
    while(std::getline(infile, line)) {
        int filePointer = 0;
        int nextPointer = line.find_first_of(",", filePointer);
    }
    auto startPose = points->getPose(ValorPoints::LOCATIONS::START, 92);
    auto endPose = points->getPose(ValorPoints::LOCATIONS::BUGS, 90);
    std::vector<frc::Pose2d> poses{startPose, endPose};
    auto command = createTrajectoryCommand(createTrajectory(poses));
}