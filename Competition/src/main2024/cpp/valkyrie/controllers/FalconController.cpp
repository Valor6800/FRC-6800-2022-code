#include "valkyrie/controllers/FalconController.h"

#define FALCON_TICKS_PER_REV 2048

using namespace valor;

FalconController::FalconController(int canID,
                                             valor::NeutralMode _mode,
                                             bool _inverted,
                                             std::string canbus) :
    BaseController(new WPI_TalonFX{canID, canbus}, _inverted, _mode)
{
    init();
}

void FalconController::init()
{
    motor->ConfigFactoryDefault();
    motor->SetInverted(inverted);
    setNeutralMode(neutralMode);

    motor->EnableVoltageCompensation(true);
    motor->ConfigVoltageCompSaturation(10);
    motor->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 60, 80, .75)); //potentially could do 40 60

    motor->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
    motor->ConfigAllowableClosedloopError(0, 0);
    motor->Config_IntegralZone(0, 0);

    motor->ConfigNeutralDeadband(0.01);

    valor::PIDF motionPIDF;
    setPIDF(motionPIDF, 0);
    reset();

    wpi::SendableRegistry::AddLW(this, "FalconController", "ID " + std::to_string(motor->GetDeviceID()));
}

void FalconController::reset()
{
    motor->SetSelectedSensorPosition(0);
}

void FalconController::setEncoderPosition(double position)
{
    motor->SetSelectedSensorPosition(position / conversion * FALCON_TICKS_PER_REV, 0);
}

void FalconController::setupFollower(int canID, bool followerInverted)
{
    followerMotor = new WPI_TalonFX(canID);
    followerMotor->Follow(*motor);
    if (followerInverted) {
        followerMotor->SetInverted(!motor->GetInverted());
    }
    setNeutralMode(BaseController::neutralMode);
}

void FalconController::setForwardLimit(double forward)
{
    double rawForward = forward / conversion * FALCON_TICKS_PER_REV;
    motor->ConfigForwardSoftLimitThreshold(rawForward);
    motor->ConfigForwardSoftLimitEnable(true);
}

void FalconController::setReverseLimit(double reverse)
{
    double rawReverse = reverse / conversion * FALCON_TICKS_PER_REV;
    motor->ConfigReverseSoftLimitThreshold(rawReverse);
    motor->ConfigReverseSoftLimitEnable(true);
}

void FalconController::setPIDF(valor::PIDF _pidf, int slot)
{
    pidf = _pidf;
    motor->Config_kP(slot, pidf.P);
    motor->Config_kI(slot, pidf.I);
    motor->Config_kD(slot, pidf.D);
    motor->Config_kF(slot, pidf.F * (1023.0 / 7112.0));
    motor->ConfigAllowableClosedloopError(slot, pidf.error * FALCON_TICKS_PER_REV / conversion);
    double vel = pidf.velocity / 10.0 * FALCON_TICKS_PER_REV / conversion;
    motor->ConfigMotionCruiseVelocity(vel);
    motor->ConfigMotionAcceleration(vel / pidf.acceleration);
    motor->ConfigMotionSCurveStrength(pidf.sCurveStrength);
}

void FalconController::setConversion(double _conversion)
{
    conversion = _conversion;
}

double FalconController::getCurrent()
{
    return motor->GetOutputCurrent();
}

/**
 * Get the position in units (specified by conversion)
 */
double FalconController::getPosition()
{
    return motor->GetSelectedSensorPosition() * conversion / FALCON_TICKS_PER_REV;
}

double FalconController::getSpeed()
{
    return motor->GetSelectedSensorVelocity() * 10 * conversion / FALCON_TICKS_PER_REV;
}

void FalconController::setRange(int slot, double min, double max)
{
    
}

void FalconController::setPosition(double position)
{
    if (pidf.aFF != 0) {
        double horizontalOffset = getPosition() - pidf.aFFTarget;
        double scalar = std::cos(horizontalOffset * M_PI / 180.0);
        motor->Set(ControlMode::MotionMagic, position / conversion * FALCON_TICKS_PER_REV, DemandType_ArbitraryFeedForward, scalar * pidf.aFF);
    } else
        motor->Set(ControlMode::MotionMagic, position / conversion * FALCON_TICKS_PER_REV);
}

void FalconController::setSpeed(double speed)
{
    motor->Set(ControlMode::Velocity, speed / 10 / conversion * FALCON_TICKS_PER_REV);
}

void FalconController::setPower(double speed)
{
    motor->Set(ControlMode::PercentOutput, speed);
}

void FalconController::setProfile(int profile)
{
    motor->SelectProfileSlot(profile, 0);
}

void FalconController::preventBackwards()
{
    motor->ConfigPeakOutputReverse(0);
}

double FalconController::getAbsEncoderPosition()
{
    return 0;
}

void FalconController::setNeutralMode(valor::NeutralMode mode){
    motor->SetNeutralMode(mode == valor::NeutralMode::Brake ?
                        ctre::phoenix::motorcontrol::NeutralMode::Brake :
                        ctre::phoenix::motorcontrol::NeutralMode::Coast);
    neutralMode = mode;
}

void FalconController::setVoltageCompensation(double voltage){
    motor->ConfigVoltageCompSaturation(voltage);
}

void FalconController::setOpenLoopRamp(double time){
    motor->ConfigOpenloopRamp(time);
}

void FalconController::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty(
        "Amps", 
        [this] { return getCurrent(); },
        nullptr);
    builder.AddDoubleProperty(
        "Position", 
        [this] { return getPosition(); },
        nullptr);
    builder.AddDoubleProperty(
        "Speed", 
        [this] { return getSpeed(); },
        nullptr);
    builder.AddBooleanProperty(
        "Inverted", 
        [this] { return inverted; },
        nullptr);
    builder.AddDoubleProperty(
        "Out Volt", 
        [this] { return motor->GetMotorOutputVoltage(); },
        nullptr);
}