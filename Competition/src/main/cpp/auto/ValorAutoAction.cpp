#include "auto/ValorAutoAction.h"

// split a string by commas
std::vector<std::string> ValorAutoAction::parseCSVLine(std::string line)
{
    int pointerPos = 0;
    std::vector<std::string> items;

    while (pointerPos >= 0) {
        int returnPos = line.find_first_of(",", pointerPos);
        if (returnPos > 0) {
            items.push_back(line.substr(pointerPos,returnPos-pointerPos));
            pointerPos = returnPos + 2;
        } else {
            items.push_back(line.substr(pointerPos));
            pointerPos = -1;
        }
    }
    return items;
}

frc::Pose2d ValorAutoAction::getPose(frc::Translation2d position, double angle)
{
    return frc::Pose2d(position, frc::Rotation2d(units::degree_t{angle}));
}

ValorAutoAction::ValorAutoAction(std::string line, std::map<std::string, frc::Translation2d> * points)
{
    std::vector<std::string> items = parseCSVLine(line);
    
    if (items.empty()) {
        type = ValorAutoAction::Type::NONE;
        return;
    } else if (items[0] == "time") {
        type = ValorAutoAction::Type::TIME;
    } else if (items[0] == "state") {
        type = ValorAutoAction::Type::STATE;
    } else if (items[0] == "trajectory") {
        type = ValorAutoAction::Type::TRAJECTORY;
    }

    if (type == ValorAutoAction::Type::TIME) {
        if (items.size() < 2) {
            error = ValorAutoAction::Error::SIZE_MISMATCH;
            return;
        }
        duration_ms = atoi(items[1].c_str());
        return;
    }
    else if (type == ValorAutoAction::Type::STATE) {
        if (items.size() < 3) {
            error = ValorAutoAction::Error::SIZE_MISMATCH;
            return;
        }
        state = items[1];
        value = items[2];
    }
    else if (type == ValorAutoAction::Type::TRAJECTORY) {
        if (items.size() < 4) {
            error = ValorAutoAction::Error::SIZE_MISMATCH;
            return;
        }

        if (points->count(items[1]) == 0 || points->count(items[2]) == 0) {
            error = ValorAutoAction::Error::POINT_MISSING;
            return;
        }

        auto _start = points->at(items[1]);
        auto _end = points->at(items[2]);

        start = getPose(_start, atoi(items[3].c_str()));
        end = getPose(_end, atoi(items[3].c_str()));
    }
}