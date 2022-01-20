#include "ValorAuto.h"

ValorAuto::ValorAuto(Drivetrain *_drivetrain) : drivetrain(_drivetrain)
{    
    // See: https://github.com/wpilibsuite/allwpilib/blob/v2022.1.1/wpilibcExamples/src/main/cpp/examples/SwerveControllerCommand/cpp/RobotContainer.cpp

    frc::TrajectoryConfig config(units::velocity::meters_per_second_t{SwerveConstants::AUTO_MAX_SPEED_MPS},
                                 units::acceleration::meters_per_second_squared_t{SwerveConstants::AUTO_MAX_ACCEL_MPSS});

    config.SetKinematics(drivetrain->getKinematics());

    frc::TrajectoryConfig reverseConfig(units::velocity::meters_per_second_t{SwerveConstants::AUTO_MAX_SPEED_MPS},
                                    units::acceleration::meters_per_second_squared_t{SwerveConstants::AUTO_MAX_ACCEL_MPSS});

    reverseConfig.SetKinematics(drivetrain->getKinematics());
    reverseConfig.SetReversed(true);

    frc::ProfiledPIDController<units::radians> thetaController{
            DriveConstants::KPT,
            DriveConstants::KIT,
            DriveConstants::KDT,
            frc::ProfiledPIDController<units::radians>::Constraints(
                units::angular_velocity::radians_per_second_t{SwerveConstants::AUTO_MAX_ROTATION_RPS},
                units::angular_acceleration::radians_per_second_squared_t{SwerveConstants::AUTO_MAX_ROTATION_ACCEL_RPSS})
    };

    // @TODO look at angle wrapping and modding
    thetaController.EnableContinuousInput(units::radian_t(-wpi::numbers::pi),
                                        units::radian_t(wpi::numbers::pi));

frc::Pose2d bugs = frc::Pose2d(0.7_m, 1.234_m, frc::Rotation2d(0_deg));
frc::Pose2d daffy = frc::Pose2d(2.0_m,.057_m, frc::Rotation2d(0_deg));
frc::Pose2d porky = frc::Pose2d(6_m, 0.19_m, frc::Rotation2d(20_deg));
frc::Pose2d shoot = frc::Pose2d(2.626_m,0_m, frc::Rotation2d(-90_deg));

    auto moveBugs = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        {frc::Translation2d{0_m,1.234_m}},
        bugs,
        config);

    auto movePorky = frc::TrajectoryGenerator::GenerateTrajectory(
        bugs,
        {frc::Translation2d{2.0_m, 0.057_m}},
        porky,
        config);
    auto movePorkyFromDaffy = frc::TrajectoryGenerator::GenerateTrajectory(
        daffy,
        {},
        porky,
        config);

    auto moveDaffy = frc::TrajectoryGenerator::GenerateTrajectory(
        bugs,
        {},
        daffy,
        config);

    auto moveShoot = frc::TrajectoryGenerator::GenerateTrajectory(
        porky,
        {},
        shoot,
        reverseConfig);
    
    frc2::SwerveControllerCommand<4> cmd_move_moveBugs(
        moveBugs,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveDaffy(
        moveDaffy,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );
    

    frc2::SwerveControllerCommand<4> cmd_move_movePorky(
        movePorky,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );
    frc2::SwerveControllerCommand<4> cmd_move_movePorkyFromDaffy(
        movePorkyFromDaffy,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveShoot(
        moveShoot,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

/*
    frc2::SequentialCommandGroup *shoot4 = new frc2::SequentialCommandGroup();
    shoot4->AddCommands
    (cmd_move_move1,
    cmd_move_move2,
    cmd_move_move3); */

     frc2::SequentialCommandGroup *shoot4New = new frc2::SequentialCommandGroup();
    shoot4New->AddCommands
    (cmd_move_moveBugs,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_move_movePorky,
    cmd_move_moveShoot);

    frc2::SequentialCommandGroup *shoot5 = new frc2::SequentialCommandGroup();
    shoot5->AddCommands
    (cmd_move_moveBugs,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_move_moveDaffy,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_move_movePorkyFromDaffy,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_move_moveShoot);  

    frc2::SequentialCommandGroup *shoot5RemoveTaz = new frc2::SequentialCommandGroup();
    shoot5RemoveTaz->AddCommands
    (cmd_move_moveBugs,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_move_moveDaffy,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_move_movePorkyFromDaffy,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_move_moveShoot); 
/*
    frc2::SequentialCommandGroup *leaveTarmac = new frc2::SequentialCommandGroup();
    leaveTarmac->AddCommands
    (cmd_move_move1);

    m_chooser.AddOption("basic movement auto", leaveTarmac);
    m_chooser.AddOption("4 ball auto", shoot4);
*/
    m_chooser.SetDefaultOption("4 ball auto new", shoot4New);
    frc::SmartDashboard::PutData(&m_chooser);
}

frc2::Command* ValorAuto::getCurrentAuto() {
    return m_chooser.GetSelected();
    }