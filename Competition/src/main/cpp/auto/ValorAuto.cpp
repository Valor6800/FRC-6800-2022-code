#include "auto/ValorAuto.h"
#include "Constants.h"
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <map>
#include <functional>
#include <vector>
#include <wpi/ghc/filesystem.hpp>

#include <frc2/command/CommandBase.h>

frc::TrajectoryConfig ValorAuto::config(
    units::velocity::meters_per_second_t{SwerveConstants::AUTO_MAX_SPEED_MPS},
    units::acceleration::meters_per_second_squared_t{SwerveConstants::AUTO_MAX_ACCEL_MPSS});

ValorAuto::ValorAuto(Drivetrain *_drivetrain, Shooter *_shooter, Feeder *_feeder, TurretTracker *_turretTracker) :
    drivetrain(_drivetrain), 
    shooter(_shooter),
    feeder(_feeder),
    turretTracker(_turretTracker)
{
    ValorAuto::config.SetKinematics(drivetrain->getKinematics());

    // @TODO look at angle wrapping and modding
    thetaController.EnableContinuousInput(units::radian_t(-wpi::numbers::pi),
                                          units::radian_t(wpi::numbers::pi));
}

// directory_iterator doesn't exist in vanilla c++11, luckily wpilib has it in their library
// https://first.wpi.edu/wpilib/allwpilib/docs/release/cpp/classghc_1_1filesystem_1_1directory__iterator.html
// implementation
// https://stackoverflow.com/questions/62409409/how-to-make-stdfilesystemdirectory-iterator-to-list-filenames-in-order
std::vector<std::string> listDirectory(std::string path_name){
    std::vector<std::string> files;

    for (auto &entry : ghc::filesystem::directory_iterator(path_name))
        files.push_back(entry.path());
    return files;
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

/* Read in the points from a CSV file.
 * Each line in the CSV file must be written in the following format:
`point_name,point_x,point_y`
 * Example points CSV file:
 ```Bugs,5,2.5
    F1,8,9
    start,0,0```
 * Note that there are no spaces.
 * There are no restricitions on what points must be included in the file - just
   make sure that the points used in whatever auto you are using also exist in this CSV file.
 */
void ValorAuto::readPointsCSV(std::string filename){
    std::ifstream infile(filename);
    if (!infile.good()){
        return;
    }

    points.clear();

    std::string line; 
    while (std::getline(infile, line)){
        std::vector<std::string> items = ValorAutoAction::parseCSVLine(line);

        // empty or invalid line
        if (items.size() != 3)
            continue;

        std::string name = items[0];
        double x = std::stod(items[1]), y = std::stod(items[2]);
        points[name] = frc::Translation2d((units::length::meter_t)x, (units::length::meter_t)y);
    }
}

frc2::SequentialCommandGroup && makeRValue(frc2::SequentialCommandGroup group){
    return group;
}

frc2::SequentialCommandGroup* ValorAuto::makeAuto(std::string filename){
    entry.SetString("creating command group");
    frc2::SequentialCommandGroup *cmdGroup = new frc2::SequentialCommandGroup();

    // @TODO Add a new ValorAutoAction type responsible for handling this

    std::ifstream infile(filename);
    if (!infile.good()){
        return cmdGroup;
    }

    std::string line;

    while (std::getline(infile, line)){
        ValorAutoAction action(line, &points);
        if (action.type == ValorAutoAction::Type::TRAJECTORY){
            frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
                action.start,
                {},
                action.end,
                config
            );
            cmdGroup->AddCommands(createTrajectoryCommand(trajectory));
        }
        else if (action.type == ValorAutoAction::Type::STATE){
            std::function<void(void)> func;

            if (action.state == "flywheelState"){
                func = [&, action] {
                    shooter->state.flywheelState = shooter->stringToFlywheelState(action.value);
                };
            }
            else if (action.state == "turretState"){
                func = [&, action] {
                    shooter->state.turretState = shooter->stringToTurretState(action.value);
                };
            }
            else if (action.state == "hoodState"){
                func = [&, action] {
                    shooter->state.hoodState = shooter->stringToHoodState(action.value);
                };
            }
            else if (action.state == "feederState"){
                func = [&, action] {
                    feeder->state.feederState = feeder->stringToFeederState(action.value);
                };
            }
            cmdGroup->AddCommands(frc2::InstantCommand(func));

        }
        else if (action.type == ValorAutoAction::Type::TIME){
            cmdGroup->AddCommands(frc2::WaitCommand((units::millisecond_t)action.duration_ms));
        }
        else if (action.type == ValorAutoAction::Type::RESET_ODOM){
            cmdGroup->AddCommands(
                frc2::InstantCommand(
                    [&, action] {
                        drivetrain->resetOdometry(action.start);
                    }
                )
            );
        }
        else if (action.type == ValorAutoAction::Type::ACTION){
            /*
            This doesn't work for a specific reason.
            If you look into SequentialCommandGroup.h, you'll find the following:

            SequentialCommandGroup(SequentialCommandGroup&& other) = default;
            SequentialCommandGroup(const SequentialCommandGroup&) = delete;
            SequentialCommandGroup(SequentialCommandGroup&) = delete;

            You'll notice that functions taking in SequentialCommandGroup& are unavailable, which is exactly what we're passing in
            However, SequeuentialCommandGroup&& is available.
            This means that rvalue SequentialCommandGroups can be passed in.
            rvalue are values that have no memory address.
            Your typical values, defined by something like
            int a = 5;
            are lvalues. They are an object reference, as compared to an rvalue, which is just a value, not related to any object.
            So in this scenario, if you were to access a, that would be an lvalue,
            but if you acccessed 5, that would be an rvalue
            rvalues are defined with
            int&& a = 5;


            So either you create the command group right here, or you need to find a way to convert
            lvalues (the typical ones) to rvalues
            */

            // The issue is not with the pointers!!
            // cmdGroup->AddCommands((*precompiledActions[action.name]));
            frc2::SequentialCommandGroup grp{};
            // cmdGroup->AddCommands(grp); // This doesn't compile!

            // ...which is why this code works
            
            cmdGroup->AddCommands(frc2::SequentialCommandGroup{
                frc2::InstantCommand(
                    [&, action] {
                        drivetrain->resetOdometry(frc::Pose2d(0_m, 0_m, 0_deg));
                    }
                )
            });
            
        }
    }
    return cmdGroup;
}

void ValorAuto::precompileActions(std::string path_name){
    precompiledActions.clear();
    std::vector<std::string> action_paths = listDirectory(path_name);
    for (std::string action_path: action_paths){
        frc2::SequentialCommandGroup * cmdGroup = makeAuto(action_path);
        std::string name = action_path.substr(action_path.find_last_of('/') + 1, action_path.find_last_of('.') - action_path.find_last_of('/') - 1);
        
        precompiledActions[name] = (cmdGroup);
    }
}

bool is_alpha(char c){
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
}
bool is_caps(char c){
    return (c >= 'A' && c <= 'Z');
}

std::string makeFriendlyName(std::string filename){
    // take last part of the path string when divided with /'s - this should be the filename
    filename = filename.substr(filename.find_last_of('/') + 1);
    std::string n_name = "";
    for (int i = 0; i < filename.length(); i ++){
        // .'s signify the end of the filename and the start of the file extension
        if (filename[i] == '.'){
            break;
        } else if (filename[i] == '_'){ // replace _'s with spaces for a snake case filename
            // make sure we dont have double spaces
            if (*(n_name.end() - 1) != ' ')
                n_name += ' ';
        } else if (i >= 0 && is_alpha(filename[i]) && is_caps(filename[i]) && !is_caps(filename[i - 1]) && *(n_name.end() - 1) != ' '){ // check for camel case, add space if present
            n_name += ' ';
            n_name += tolower(filename[i]);
        } else if (i == 0){ // first letter should be capitaized
            n_name += toupper(filename[i]);
        } else{
            n_name += tolower(filename[i]);
        }
    }
    return n_name;
}

frc2::SequentialCommandGroup * ValorAuto::getCurrentAuto(){
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("datatable");
    entry = table->GetEntry("log");
    entry.SetString("reading points");
    readPointsCSV("/home/lvuser/test_points.csv");
    precompileActions("/home/lvuser/actions/");

    return makeAuto(m_chooser.GetSelected());
}

void ValorAuto::fillAutoList(){
    std::string autos_path = "/home/lvuser/auto_csvs/";
    std::vector<std::string> avAutos = listDirectory(autos_path);
    for (std::string a: avAutos){
        m_chooser.AddOption(makeFriendlyName(a), a);
    }
    frc::SmartDashboard::PutData(&m_chooser);
}