#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <map>

#ifndef VALOR_POINTS_H
#define VALOR_POINTS_H

class ValorPoints {

public:

    enum ALLIANCE_COLOR {
        RED,
        BLUE,
    };

    enum LOCATIONS {
        START,
        BUGS,
        BACK_BUGS,
    };

    ValorPoints(ALLIANCE_COLOR);

    frc::Translation2d getTranslation(LOCATIONS);
    frc::Pose2d getPose(LOCATIONS, double);

    void setColor(ALLIANCE_COLOR);

private:

    frc::Translation2d xy(double, double);

    ALLIANCE_COLOR color;
    std::map<ALLIANCE_COLOR, std::map<LOCATIONS, frc::Translation2d>> autoMap;
};

#endif
