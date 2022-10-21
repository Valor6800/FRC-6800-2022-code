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

// read points from a csv file 
void ValorAuto::readPointsCSV(std::string filename){
    std::ifstream infile(filename);
    if (!infile.good()){
        return;
    }

    points->clear();

    std::string line; 
    while (std::getline(infile, line)){
        std::vector<std::string> items = ValorAutoAction::parseCSVLine(line);

        // empty line
        if (items.size() == 0)
            continue;

        std::string name = items[0];
        double x = std::stod(items[1]), y = std::stod(items[2]);
        points->insert({name, frc::Translation2d((units::length::meter_t)x, (units::length::meter_t)y)});
    }
}

// TODO: Please figure out something better
Shooter::FlywheelState fromStringFlywheelEnum(const std::string& str){
    if (str == "FLYWHEEL_DISABLE")
        return Shooter::FlywheelState::FLYWHEEL_DISABLE;
    else if (str == "FLYWHEEL_DEFAULT")
        return Shooter::FlywheelState::FLYWHEEL_DEFAULT;
    else if (str == "FLYWHEEL_TRACK")
        return Shooter::FlywheelState::FLYWHEEL_TRACK;
    else if (str == "FLYWHEEL_POOP")
        return Shooter::FlywheelState::FLYWHEEL_POOP;
}

Shooter::TurretState fromStringTurretEnum(const std::string& str) {
    if (str == "TURRET_DISABLE")
        return Shooter::TurretState::TURRET_DISABLE;
    else if (str == "TURRET_MANUAL")
        return Shooter::TurretState::TURRET_MANUAL;
    else if (str == "TURRET_HOME_MID")
        return Shooter::TurretState::TURRET_HOME_MID;
    else if (str == "TURRET_HOME_LEFT")
        return Shooter::TurretState::TURRET_HOME_LEFT;
    else if (str == "TURRET_HOME_RIGHT")
        return Shooter::TurretState::TURRET_HOME_RIGHT;
    else if (str == "TURRET_TRACK")
        return Shooter::TurretState::TURRET_TRACK;
}

Shooter::HoodState fromStringHoodEnum(const std::string& str) {
    if (str == "HOOD_DOWN")
        return Shooter::HoodState::HOOD_DOWN;
    else if (str == "HOOD_TRACK")
        return Shooter::HoodState::HOOD_TRACK;
    else if (str == "HOOD_POOP")
        return Shooter::HoodState::HOOD_POOP;
}

Feeder::FeederState fromStringFeederEnum(const std::string& str){
    if (str == "FEEDER_DISABLE")
        return Feeder::FeederState::FEEDER_DISABLE;
    else if (str == "FEEDER_REVERSE")
        return Feeder::FeederState::FEEDER_REVERSE;
    else if (str == "FEEDER_SHOOT")
        return Feeder::FeederState::FEEDER_SHOOT;
    else if (str == "FEEDER_CURRENT_INTAKE")
        return Feeder::FeederState::FEEDER_CURRENT_INTAKE;
    else if (str == "FEEDER_REGULAR_INTAKE")
        return Feeder::FeederState::FEEDER_REGULAR_INTAKE;
}


frc2::SequentialCommandGroup* ValorAuto::makeAuto(std::string filename){
    frc2::SequentialCommandGroup *cmdGroup = new frc2::SequentialCommandGroup();

    std::ifstream infile(filename);
    if (!infile.good()){
        return cmdGroup;
    }
    
    std::string line;

    while (std::getline(infile, line)){
        ValorAutoAction *action = new ValorAutoAction(line, points);
        if (action->type == ValorAutoAction::Type::TRAJECTORY){
            frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
                action->start,
                {},
                action->end,
                config
            );
            cmdGroup->AddCommands(createTrajectoryCommand(trajectory));
        }
        else if (action->type == ValorAutoAction::Type::STATE){
            std::function<void(void)> func;
            
            // make value string uppercase
            std::string value = "";
            for (char c: action->value)
                value += toupper(c);

            if (action->state == "flywheelState"){
                func = [&] {
                    shooter->state.flywheelState = fromStringFlywheelEnum(value);
                };
            }
            else if (action->state == "turretState"){
                func = [&] {
                    shooter->state.turretState = fromStringTurretEnum(value);
                };
            }
            else if (action->state == "hoodState"){
                func =  [&] {
                    shooter->state.hoodState = fromStringHoodEnum(value);
                };
            }
            else if (action->state == "feederState"){
                func =  [&] {
                    feeder->state.feederState = fromStringFeederEnum(value);
                };
            }
            cmdGroup->AddCommands(frc2::InstantCommand(func));

        }
        else if (action->type == ValorAutoAction::Type::TIME){
            cmdGroup->AddCommands(frc2::WaitCommand((units::second_t)stod(action->value)));
        }
    }
    return cmdGroup;
}

bool is_alpha(char c){
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
}
bool is_caps(char c){
    return (c >= 'A' && c <= 'Z');
}
char to_lower(char c){
    return (c >= 'A' && c <= 'Z') ? c - 'A' + 'a' : c;
}
char to_upper(char c){
    return (c >= 'a' && c <= 'z') ? c - 'a' + 'A' : c;
}

std::string makeFriendlyName(std::string filename){
    std::string n_name = "";
    for (int i = 0; i < filename.length(); i ++){
        if (filename[i] == '.'){
            break;
        } else if (filename[i] == '_'){
            if (*(n_name.end() - 1) != ' ')
                n_name += ' ';
        } else if (i >= 0 && is_alpha(filename[i]) && is_caps(filename[i]) && !is_caps(filename[i - 1]) && *(n_name.end() - 1) != ' '){
            n_name += ' ';
            n_name += to_lower(filename[i]);
        } else if (i == 0){
            n_name += to_upper(filename[i]);
        } else{
            n_name += to_lower(filename[i]);
        }
    }
    return n_name;
}

// https://first.wpi.edu/wpilib/allwpilib/docs/release/cpp/classghc_1_1filesystem_1_1directory__iterator.html
std::vector<std::string> listDirectory(std::string path_name){
    std::vector<std::string> files;

    for (auto &entry : ghc::filesystem::directory_iterator(path_name))
        files.push_back(entry.path());
    return files;
}

frc2::SequentialCommandGroup * ValorAuto::getCurrentAuto(){
    readPointsCSV("test_points.csv");
    return makeAuto(m_chooser.GetSelected());
}

void ValorAuto::fillAutoList(){
    std::string autos_path = "/auto_csvs/";
    std::vector<std::string> avAutos = listDirectory(autos_path);
    for (std::string a: avAutos){
        m_chooser.AddOption(makeFriendlyName(a), a);
    }
}