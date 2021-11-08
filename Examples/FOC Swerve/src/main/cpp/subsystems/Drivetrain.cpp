/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Drivetrain.h"
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>

Drivetrain::Drivetrain() : ValorSubsystem(),
                           driverController(NULL),
                           navX(frc::SerialPort::Port::kMXP),
                           hasGyroOffset(false),
                           kinematics(motorLocations[0], motorLocations[1], motorLocations[2], motorLocations[3]),
                           odometry(kinematics, frc::Rotation2d{units::radian_t{0}}) {
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Drivetrain::~Drivetrain()
{
    for (int i = 0; i < 4; i++) {
        delete azimuthMotors[i];
        delete driveMotors[i];
        delete swerveModules[i];
        delete magEncoders[i];
    }
}

void Drivetrain::configSwerveModule(int i)
{
    azimuthMotors[i] = new WPI_TalonFX(DriveConstants::AZIMUTH_CANS[i]);
    azimuthMotors[i]->ConfigFactoryDefault();
    // azimuthMotors[i]->SetInverted(true);
    azimuthMotors[i]->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
    azimuthMotors[i]->ConfigAllowableClosedloopError(0, 0);
    azimuthMotors[i]->Config_IntegralZone(0, 0);
    azimuthMotors[i]->Config_kF(0, 0);
    azimuthMotors[i]->Config_kD(0, SwerveConstants::KD);
    azimuthMotors[i]->Config_kI(0, 0);
    azimuthMotors[i]->Config_kP(0, SwerveConstants::KP);
    azimuthMotors[i]->ConfigMotionAcceleration(SwerveConstants::MOTION_ACCELERATION);
    azimuthMotors[i]->ConfigMotionCruiseVelocity(SwerveConstants::MOTION_CRUISE_VELOCITY);
    
    driveMotors[i] = new WPI_TalonFX(DriveConstants::DRIVE_CANS[i]);
    driveMotors[i]->ConfigFactoryDefault();
    // driveMotors[i]->SetInverted(true);
    driveMotors[i]->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
    driveMotors[i]->SetNeutralMode(NeutralMode::Brake);

    magEncoders.push_back(new TalonSRX(DriveConstants::MAG_ENCODER_CANS[i]));
    magEncoders[i]->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);

    swerveModules.push_back(new ValorSwerve(azimuthMotors[i], driveMotors[i], magEncoders[i], motorLocations[i]));
}

void Drivetrain::init() {
    initTable("Drivetrain");
    navX.Calibrate();

    for (int i = 0; i < 4; i++) {
        configSwerveModule(i);
    }
}

void Drivetrain::setController(frc::XboxController* controller) {
    driverController = controller;
}

std::vector<ValorSwerve*> Drivetrain::getSwerveModules()
{
    return swerveModules;
}

void Drivetrain::assessInputs() {
    if (!driverController) {
        return;
    }

    // driver inputs
    state.leftStickX = driverController->GetX(frc::GenericHID::kLeftHand);
    state.leftStickY = driverController->GetY(frc::GenericHID::kLeftHand);
    state.rightStickX = driverController->GetX(frc::GenericHID::kRightHand);
    state.rightStickY = driverController->GetY(frc::GenericHID::kRightHand);

    odometry.Update(getHeading(),
                    swerveModules[0]->getState(),
                    swerveModules[1]->getState(),
                    swerveModules[2]->getState(),
                    swerveModules[3]->getState());
}

void Drivetrain::analyzeDashboard()
{
    table->PutNumber("Robot X", getPose_m().X().to<double>());
    table->PutNumber("Robot Y", getPose_m().Y().to<double>());

    table->PutNumber("Gyro rate", getGyroRate());
    table->PutNumber("Gyro angle", getGyroAngle());

    for (int i = 0; i < 4; i++) {
        table->PutNumber("Wheel " + std::to_string(i) + " angle", swerveModules[i]->getAzimuthRotation2d().Degrees().to<double>());
        table->PutNumber("Wheel " + std::to_string(i) + " X", swerveModules[i]->getWheelLocation_m().X().to<double>());
        table->PutNumber("Wheel " + std::to_string(i) + " Y", swerveModules[i]->getWheelLocation_m().Y().to<double>());
        table->PutNumber("Wheel " + std::to_string(i) + " azimuth", swerveModules[i]->getAzimuthAbsoluteEncoderCounts());
    }
}

void Drivetrain::assignOutputs()
{
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double xSpeed = std::abs(state.leftStickY) > DriveConstants::kDeadbandY ? -1.0 * -powf(state.leftStickY, 3) : 0;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    double ySpeed = std::abs(state.leftStickX) > DriveConstants::kDeadbandX ? -powf(state.leftStickX, 3) : 0;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rot = std::abs(state.rightStickX) > DriveConstants::kDeadbandX ? powf(state.rightStickX, 3) : 0;

    drive(xSpeed, ySpeed, rot, true);
}

void Drivetrain::resetState()
{
    //@TODO zero azimuth encoders

    //Zero gyro
    resetGyro();
    double adjustment = (int)getGyroAngle() % 360;
    navX.SetAngleAdjustment(adjustment);

    drive(0, 0, 0, true);
}

frc::SwerveDriveKinematics<4> Drivetrain::getKinematics()
{
    return kinematics;
}

frc::Pose2d Drivetrain::getPose_m()
{
    return odometry.GetPose();
}

frc::Rotation2d Drivetrain::getHeading()
{
    return hasGyroOffset ? navX.GetRotation2d().RotateBy(gyroOffset) : navX.GetRotation2d();
}

double Drivetrain::getGyroAngle()
{
    //@TODO may need to apply gyro offset
    return navX.GetAngle();
}

double Drivetrain::getGyroRate()
{
    return navX.GetRate();
}

frc::Rotation2d Drivetrain::getGyroOffset()
{
    return gyroOffset;
}

void Drivetrain::setGyroOffset(frc::Rotation2d offset)
{
    if (gyroOffset == offset)
        return;
    gyroOffset = offset;
    hasGyroOffset = true;
}

void Drivetrain::resetOdometry(frc::Pose2d pose)
{
    odometry.ResetPosition(pose, navX.GetRotation2d().RotateBy(gyroOffset));
}

void Drivetrain::resetDriveEncoders()
{
    for (int i = 0; i < 4; i++) {
        swerveModules[i]->resetDriveEncoder();
    }
}

void Drivetrain::resetGyro()
{
    navX.Reset();
}

void Drivetrain::drive(double vx_mps, double vy_mps, double omega_radps, bool isFOC)
{
    auto states = getModuleStates(units::meters_per_second_t{vx_mps},
                                  units::meters_per_second_t{vy_mps},
                                  units::radians_per_second_t{omega_radps},
                                  isFOC);
    for (int i = 0; i < 4; i++) {
        swerveModules[i]->setDesiredState(states[i], true);
    }
}

void Drivetrain::move(double vx_mps, double vy_mps, double omega_radps, bool isFOC)
{
    auto states = getModuleStates(units::meters_per_second_t{vx_mps},
                                  units::meters_per_second_t{vy_mps},
                                  units::radians_per_second_t{omega_radps},
                                  isFOC);
    for (int i = 0; i < 4; i++) {
        swerveModules[i]->setDesiredState(states[i], false);
    }
}

wpi::array<frc::SwerveModuleState, 4> Drivetrain::getModuleStates(units::meters_per_second_t vx_mps,
                                                                  units::meters_per_second_t vy_mps,
                                                                  units::radians_per_second_t omega_radps,
                                                                  bool isFOC)
{
    frc::ChassisSpeeds chassisSpeeds = isFOC ? 
        frc::ChassisSpeeds::FromFieldRelativeSpeeds(vx_mps,
                                                    vy_mps,
                                                    omega_radps,
                                                    getHeading()) :
        frc::ChassisSpeeds{vx_mps,vy_mps,omega_radps};
    auto states = kinematics.ToSwerveModuleStates(chassisSpeeds);
    kinematics.NormalizeWheelSpeeds(&states, SwerveConstants::DRIVE_MAX_SPEED_MPS);
    return states;
}

void Drivetrain::setModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates)
{
    kinematics.NormalizeWheelSpeeds(&desiredStates, SwerveConstants::DRIVE_MAX_SPEED_MPS);
    for (int i = 0; i < 4; i++) {
        swerveModules[i]->setDesiredState(desiredStates[i], true);
    }
}