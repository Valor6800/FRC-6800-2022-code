#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <map>

#ifndef VALOR_POINTS_H
#define VALOR_POINTS_H

class ValorPoint {

public:

    ValorPoint();
    ValorPoint(double, double);

    void setTranslation(double, double);

    frc::Translation2d getTranslation();
    frc::Pose2d getPose(double);
private:
    frc::Translation2d position;
};

#endif
