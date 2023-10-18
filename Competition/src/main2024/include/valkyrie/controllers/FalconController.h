#pragma once

#include "valkyrie/controllers/BaseController.h"

#include <ctre/Phoenix.h>
#include <ctre/phoenix/motorcontrol/NeutralMode.h>
#include <string>

namespace valor {

class FalconController : public BaseController<WPI_TalonFX>
{
public:
    FalconController(int _canID, valor::NeutralMode _mode, bool _inverted, std::string _canbus = "");

    void init();
    void reset();
    void setNeutralMode(valor::NeutralMode mode);

    double getPosition();
    double getSpeed();
    double getCurrent();

    void setEncoderPosition(double position);
    
    void setPosition(double);
    void setSpeed(double);
    void setPower(double);

    void setupFollower(int, bool = false);
    
    void setPIDF(valor::PIDF pidf, int slot);
    void setForwardLimit(double forward);
    void setReverseLimit(double reverse);
    void setRange(int slot, double min, double max);
    
    void setConversion(double);
    void setVoltageCompensation(double);
    

    void setProfile(int slot);
    double getAbsEncoderPosition();

    /**
     * @brief Prevent the motor from traveling backwards
     * 
     * Restrict the motor from going backwards
     */
    void preventBackwards();
    
    void setOpenLoopRamp(double time);

    void InitSendable(wpi::SendableBuilder& builder);
private:
    valor::PIDF pidf;
};
}
