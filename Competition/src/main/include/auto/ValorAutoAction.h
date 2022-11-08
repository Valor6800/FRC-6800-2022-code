#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <map>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <string>

#ifndef VALOR_AUTO_ACTION_H
#define VALOR_AUTO_ACTION_H

struct ValorAutoAction {
    enum Type {
        NONE,
        TIME,
        STATE,
        TRAJECTORY,
        RESET_ODOM,
        ACTION
    } type;

    enum Error {
        NONE_ERROR, // can't have duplicate enum names
        SIZE_MISMATCH,
        POINT_MISSING
    } error;

    frc::Pose2d start;
    frc::Pose2d end;

    std::string state;
    std::string value;
    
    int duration_ms;

    ValorAutoAction(std::string line, std::map<std::string, frc::Translation2d> *);

    // @TODO Find a better way to implement this
    // The issue with InstantCommand and WaitCommand
    // is that they both inherit from an abstract class
    // You cannot directly create objects of an abstract class,
    // so when trying to iterate through the vector 
    // (or put anything in it) it yells at you.
    // I've looked into ways to have multiple types in a single vector
    // An answer I've found (besides using the parent class)
    // is unions
    // UPDATE 11/3/2022
    // You can add a command group to another command group, rendering a vector of commands irrelevant
    // UPDATE 11/3/2022
    // This is now stored in the auto class, rendering it irrelevant
    // frc2::SequentialCommandGroup * cmdGroup;

    std::string name;
    

public:
    static std::vector<std::string> parseCSVLine(std::string);

private:
    frc::Pose2d getPose(frc::Translation2d, double);
};

#endif