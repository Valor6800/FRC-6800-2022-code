#include "auto/ValorPoint.h"

ValorPoint::ValorPoint()
{
    setTranslation(0, 0);
}

ValorPoint::ValorPoint(double x, double y)
{
    setTranslation(x, y);
}

void ValorPoint::setTranslation(double x, double y)
{
    position = frc::Translation2d(units::meter_t{x}, units::meter_t{y});
}

frc::Translation2d ValorPoint::getTranslation()
{
    return position;
}

frc::Pose2d ValorPoint::getPose(double angle)
{
    return frc::Pose2d(position, frc::Rotation2d(units::degree_t{angle}));
}
