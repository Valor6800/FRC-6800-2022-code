#include "auto/ValorPoints.h"

ValorPoints::ValorPoints(ALLIANCE_COLOR _color) : color(_color)
{
    autoMap[RED][START] = xy(7, 1.771);
    autoMap[BLUE][START] = xy(7, 1.771);

    autoMap[RED][BUGS] = xy(7, 0.3);
    autoMap[BLUE][BUGS] = xy(7, 0.3);

    autoMap[RED][BACK_BUGS] = xy(7, 1.2);
    autoMap[BLUE][BACK_BUGS] = xy(7, 1.2);
}

frc::Translation2d ValorPoints::xy(double x, double y)
{
    return frc::Translation2d(units::meter_t{x}, units::meter_t{y});
}

void ValorPoints::setColor(ALLIANCE_COLOR _color)
{
    color = _color;
}

frc::Translation2d ValorPoints::getTranslation(ValorPoints::LOCATIONS location)
{
    return autoMap[color][location];
}

frc::Pose2d ValorPoints::getPose(ValorPoints::LOCATIONS location, double angle)
{
    return frc::Pose2d(autoMap[color][location], frc::Rotation2d(units::degree_t{angle}));
}
