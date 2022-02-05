#include "ValorAuto.h"

ValorAuto::ValorAuto(Drivetrain *_drivetrain, Shooter *_shooter, Feeder *_feeder) : 
    drivetrain(_drivetrain), 
    shooter(_shooter),
    feeder(_feeder)
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

frc::Pose2d startPose = frc::Pose2d(8.514_m, 1.771_m, frc::Rotation2d(182.1_deg));

frc::Translation2d preBugs{startPose.X(), 0.28_m};
frc::Pose2d bugs = frc::Pose2d(7.559_m, 0.28_m, frc::Rotation2d(180_deg));
frc::Pose2d daffy = frc::Pose2d(5.083_m, 1.867_m, frc::Rotation2d(155_deg));
frc::Pose2d porky = frc::Pose2d(0.992_m, 1.112_m, frc::Rotation2d(200_deg));
frc::Pose2d shoot = frc::Pose2d(5_m, 1.5_m, frc::Rotation2d(90_deg));



frc::Pose2d x6y4 = frc::Pose2d(6_m, 4_m, frc::Rotation2d(180_deg));
// frc::Rotation2d gyroOffsetAuto = drivetrain->getGyroOffset() + frc::Rotation2d(180_deg);


    auto move2 = frc::TrajectoryGenerator::GenerateTrajectory(
        startPose,
        {},
        x6y4,
        reverseConfig);

frc2::InstantCommand cmd_intake2 = frc2::InstantCommand( [&] { feeder->state.feederState = Feeder::FeederState::FEEDER_INTAKE2; } );

    auto moveBugs = frc::TrajectoryGenerator::GenerateTrajectory(
        startPose,
        {preBugs},
        bugs,
        config);

    auto movePorky = frc::TrajectoryGenerator::GenerateTrajectory(
        bugs,
        {frc::Translation2d{5.483_m, 1.867_m}},
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
    

    frc2::SwerveControllerCommand<4> cmd_move_move2(
        move2,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    // frc2::InstantCommand cmd_set_gyroOffset = frc2::InstantCommand( [&] {
    //     frc::Rotation2d gyroOffsetAuto = drivetrain->getGyroOffset() + frc::Rotation2d(90_deg);
    //     drivetrain->setGyroOffset(frc::Rotation2d(gyroOffsetAuto));
    //     });

    frc2::InstantCommand cmd_set_odometry = frc2::InstantCommand( [&] {
        drivetrain->resetOdometry(startPose);
        });

   

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
    (cmd_set_odometry,
    cmd_intake2,
    cmd_move_moveBugs,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_move_movePorky,
    cmd_move_moveShoot);

    frc2::SequentialCommandGroup *shoot5 = new frc2::SequentialCommandGroup();
    shoot5->AddCommands
    (cmd_set_odometry,
    cmd_intake2,
    cmd_move_moveBugs,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_move_moveDaffy,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_move_movePorkyFromDaffy,
    frc2::WaitCommand((units::second_t).5),
    cmd_move_moveShoot);  

     

    frc2::SequentialCommandGroup *move2Offset = new frc2::SequentialCommandGroup();
    move2Offset->AddCommands
    (
    cmd_set_odometry,
    cmd_move_move2); 

   

    m_chooser.SetDefaultOption("4 ball auto new", shoot4New);
    m_chooser.SetDefaultOption("5 ball auto", shoot5);
    m_chooser.AddOption("Move 2 in x Offset direction", move2Offset);

    frc::SmartDashboard::PutData(&m_chooser);
  
}

frc2::Command* ValorAuto::getCurrentAuto() {
    return m_chooser.GetSelected();
    }