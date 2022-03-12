#include "ValorAuto.h"
#include <iostream>

//See https://github.com/wpilibsuite/allwpilib/blob/v2022.1.1/wpilibcExamples/src/main/cpp/examples/SwerveControllerCommand/cpp/RobotContainer.cpp
ValorAuto::ValorAuto(Drivetrain *_drivetrain, Shooter *_shooter, Feeder *_feeder) : 
    drivetrain(_drivetrain), 
    shooter(_shooter),
    feeder(_feeder)
{   
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

    frc::Pose2d startPose = frc::Pose2d(7_m, 1.771_m, frc::Rotation2d(-92.1_deg)); //hub is 7, 2.771
    frc::Pose2d alternateStartPose = frc::Pose2d(5.566_m, 5.905_m, frc::Rotation2d(156_deg));
    frc::Pose2d endPose2ball = frc::Pose2d(10_m, 10_m, frc::Rotation2d(0_deg));

    frc::Pose2d startPose2ball = frc::Pose2d(7_m, 1.771_m, frc::Rotation2d(-92.1_deg + 240_deg)); //hub is 7, 2.771

    //Bugs y blue .35
    //bringing bugs towards the wall
    frc::Pose2d bugsBlue = frc::Pose2d(7_m, 0.3_m, frc::Rotation2d(-90_deg));
    frc::Pose2d bugsRed = frc::Pose2d(7_m, 0.3_m, frc::Rotation2d(-90_deg));

    frc::Pose2d bugsRed2ball = frc::Pose2d(7_m, 0.3_m, frc::Rotation2d(-90_deg + 240_deg));

    frc::Pose2d backBugsRed = frc::Pose2d(7_m, 1.2_m, frc::Rotation2d(-90_deg));

    frc::Pose2d rotateBlue = frc::Pose2d(6.5_m, 0.9_m, frc::Rotation2d(-150_deg));
    frc::Pose2d rotateRed = frc::Pose2d(6.5_m, 0.9_m, frc::Rotation2d(-150_deg));

    //Daffy y was 1.6
    frc::Pose2d daffyBlue = frc::Pose2d(3.55_m, 1.7_m, frc::Rotation2d(80_deg));
    
    //-3.45, -1.071 relative to hub
    //72.75
    frc::Pose2d daffyRed = frc::Pose2d(3.55_m, 2.15_m, frc::Rotation2d(115_deg));
    
    frc::Pose2d predaffyBlue = frc::Pose2d(5.083_m, .7_m, frc::Rotation2d(77_deg));
    
    //-1.917, -1.071 relative to hub
    //150.81 angle to make shot with turret to the right
    //angles appear to be 90 degrees off so use 60.81 instead
    frc::Pose2d predaffyRed = frc::Pose2d(5.083_m, .7_m, frc::Rotation2d(135_deg));
    frc::Pose2d predaffyRed2ball = frc::Pose2d(5.083_m, .7_m, frc::Rotation2d(135_deg + 240_deg));
    
    //shifting each movement by .5 to avoid smacking into the pipes
    frc::Pose2d porkyBlue = frc::Pose2d(-0.4_m, 2.35_m, frc::Rotation2d(212_deg));
    frc::Pose2d porkyRed = frc::Pose2d(-0.35_m, 2.4_m, frc::Rotation2d(212_deg));

    frc::Translation2d porkyEntryRed = frc::Translation2d(1.3_m, 3.5_m);
    frc::Translation2d porkyEntryBlue = frc::Translation2d(1.3_m, 4_m);

    frc::Pose2d porkyStepBackBlue = frc::Pose2d(.3_m, 3.35_m, frc::Rotation2d(212_deg));
    frc::Pose2d porkyStepBackRed = frc::Pose2d(.3_m, 2.8_m, frc::Rotation2d(212_deg));

    frc::Pose2d marvinBlue = frc::Pose2d(5.097_m, 6.805_m, frc::Rotation2d(156_deg));
    frc::Pose2d marvinRed = frc::Pose2d(4.066_m, 7.405_m, frc::Rotation2d(156_deg));
    
    frc::Pose2d shootBlue = frc::Pose2d(7_m, 1.2_m, frc::Rotation2d(100_deg));
    
    //0, -1.5771 relative to hub
    //+-180
    frc::Pose2d shootRed = frc::Pose2d(6_m, 1.2_m, frc::Rotation2d(65_deg));

    frc::Pose2d alternateShootBlue = frc::Pose2d(5.197_m, 6.1905_m, frc::Rotation2d(-24_deg));
    frc::Pose2d alternateShootRed = frc::Pose2d(5.317_m, 6.905_m, frc::Rotation2d(-24_deg));
    
    //frc::Translation2d preBugs{startPose.X(), bugs.Translation().Y()};

    // frc2::InstantCommand cmd_shooterPrime = frc2::InstantCommand( [&] {
    //     shooter->state.flywheelState = Shooter::FlywheelState::FLYWHEEL_PRIME; 
    //     shooter->state.hoodState = Shooter::HoodState::HOOD_UP;
    // } );

    frc2::InstantCommand cmd_shooterAuto = frc2::InstantCommand( [&] {
        shooter->state.flywheelState = Shooter::FlywheelState::FLYWHEEL_AUTO; 
        shooter->state.hoodState = Shooter::HoodState::HOOD_AUTO;
    } );

    frc2::InstantCommand cmd_turretTrack = frc2::InstantCommand( [&] {
        shooter->state.turretState = Shooter::TurretState::TURRET_TRACK;
    } );

    frc2::InstantCommand cmd_turretHomeMid = frc2::InstantCommand( [&] {
        shooter->state.turretState = Shooter::TurretState::TURRET_HOME_MID;
    } );

    frc2::InstantCommand cmd_printHeading = frc2::InstantCommand( [&] {
        std::cout << drivetrain->getPose_m().Rotation().Degrees().to<double>() << std::endl;    
    } );

    frc2::InstantCommand cmd_turretDisable = frc2::InstantCommand( [&] {
        shooter->state.turretState = Shooter::TurretState::TURRET_DISABLE;
    } );

    frc2::InstantCommand cmd_shooterDefault = frc2::InstantCommand( [&] {
        shooter->state.flywheelState = Shooter::FlywheelState::FLYWHEEL_DEFAULT; 
        shooter->state.turretState = Shooter::TurretState::TURRET_DISABLE;
        shooter->state.hoodState = Shooter::HoodState::HOOD_DOWN;
    } );

    frc2::InstantCommand cmd_intakeOne = frc2::InstantCommand( [&] { feeder->state.feederState = Feeder::FeederState::FEEDER_AUTO; } );
    frc2::InstantCommand cmd_intakeDisable = frc2::InstantCommand( [&] { feeder->state.feederState = Feeder::FeederState::FEEDER_DISABLE; } );

    frc2::InstantCommand cmd_nextBall = frc2::InstantCommand( [&] {
        shooter->state.currentBall++; 
    } );

    frc2::InstantCommand cmd_intakeAuto = frc2::InstantCommand( [&] { feeder->state.feederState = Feeder::FeederState::FEEDER_INTAKE; } );

    frc2::InstantCommand cmd_intakeShoot = frc2::InstantCommand( [&] { feeder->state.feederState = Feeder::FeederState::FEEDER_SHOOT; } );

    frc2::InstantCommand cmd_disable = frc2::InstantCommand( [&] { 
        feeder->state.feederState = Feeder::FeederState::FEEDER_DISABLE;
        shooter->state.flywheelState = Shooter::FlywheelState::FLYWHEEL_DISABLE; 
        shooter->state.turretState = Shooter::TurretState::TURRET_DISABLE;
        shooter->state.hoodState = Shooter::HoodState::HOOD_DOWN;
    });

    frc2::InstantCommand cmd_set_odometry = frc2::InstantCommand( [&] {
        drivetrain->resetOdometry(startPose);
    });

    frc2::InstantCommand cmd_set_end_2ball = frc2::InstantCommand( [&] {
        drivetrain->resetOdometry(endPose2ball);
    });

    frc2::InstantCommand cmd_set_odometry_2ball = frc2::InstantCommand( [&] {
        drivetrain->resetOdometry(startPose2ball);
    });

    frc2::InstantCommand cmd_set_alternateOdometry = frc2::InstantCommand( [&] {
        drivetrain->resetOdometry(alternateStartPose);
    });

    auto moveBugsBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        startPose,
        {},//{preBugs},
        bugsBlue,
        config);

    auto moveBugsRed = frc::TrajectoryGenerator::GenerateTrajectory(
        startPose,
        {},//{preBugs},
        bugsRed,
        config);

    auto moveBugsRed2ball = frc::TrajectoryGenerator::GenerateTrajectory(
        startPose2ball,
        {},//{preBugs},
        bugsRed2ball,
        config);

    auto moveBackBugsRed = frc::TrajectoryGenerator::GenerateTrajectory(
        bugsRed,
        {},//{preBugs},
        backBugsRed,
        reverseConfig);

    auto movePorkyBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        bugsBlue,
        {predaffyBlue.Translation()},
        porkyBlue,
        config);
    
    auto movePorkyRed = frc::TrajectoryGenerator::GenerateTrajectory(
        bugsRed,
        {predaffyRed.Translation()},
        porkyRed,
        config);

    auto movePorkyFromDaffyBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        daffyBlue,
        {porkyEntryBlue},
        porkyBlue,
        config);

    auto movePorkyFromDaffyRed = frc::TrajectoryGenerator::GenerateTrajectory(
        daffyRed,
        {porkyEntryRed},
        porkyRed,
        config);

    

    //config.SetEndVelocity(units::meters_per_second_t(2.0));
    auto moveDaffyBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        bugsBlue,
        {},
        daffyBlue,
        config);
    //config.SetEndVelocity(0);

    // int i = 4;
    // Object obj = new Object(i);
    // i = 0;

    auto moveDaffyRed = frc::TrajectoryGenerator::GenerateTrajectory(
        bugsRed,
        {},
        daffyRed,
        config);

    auto moveDaffyFromPredaffyBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        predaffyBlue,
        {},
        daffyBlue,
        config);

    auto moveDaffyFromPredaffyRed = frc::TrajectoryGenerator::GenerateTrajectory(
        predaffyRed,
        {},
        daffyRed,
        config);

    auto moveRotateBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        bugsBlue,
        {},
        rotateBlue,
        config);

    //config.SetEndVelocity(units::meters_per_second_t(1.0));
    auto moveRotateRed = frc::TrajectoryGenerator::GenerateTrajectory(
        bugsRed,
        {},
        rotateRed,
        config);
  //  config.SetEndVelocity(units::meters_per_second_t(0));

    auto movePreDaffyBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        bugsBlue, //bugsBlue,//
        {},
        predaffyBlue,
        config);
    
   // config.SetStartVelocity(units::meters_per_second_t(1.0));
    auto movePreDaffyRed = frc::TrajectoryGenerator::GenerateTrajectory(
        backBugsRed, //bugsRed,//
        {},
        predaffyRed,
        config);

    auto movePreDaffyRed2ball = frc::TrajectoryGenerator::GenerateTrajectory(
        bugsRed2ball, //bugsRed,//
        {},
        predaffyRed2ball,
        config);

        
  //  config.SetStartVelocity(units::meters_per_second_t(0));


    auto moveShootBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        porkyBlue,
        {},
        shootBlue,
        reverseConfig);

    auto moveMarvinBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        alternateStartPose,
        {},
        marvinBlue,
        config);

    auto moveMarvinRed = frc::TrajectoryGenerator::GenerateTrajectory(
        alternateStartPose,
        {},
        marvinRed,
        config);
    
    auto moveShootRed = frc::TrajectoryGenerator::GenerateTrajectory(
        porkyRed,
        {},
        shootRed, //shootRed
        reverseConfig);

    auto moveAlternateShootBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        marvinBlue,
        {},
        alternateShootBlue,
        config);

    auto moveAlternateShootRed = frc::TrajectoryGenerator::GenerateTrajectory(
        marvinRed,
        {},
        alternateShootRed,
        reverseConfig);

    auto moveStepBackBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        porkyBlue,
        {},
        porkyStepBackBlue,
        reverseConfig);

    auto moveStepBackRed = frc::TrajectoryGenerator::GenerateTrajectory(
        porkyRed,
        {},
        porkyStepBackRed,
        reverseConfig);


    frc2::SwerveControllerCommand<4> cmd_move_moveBugsBlue(
        moveBugsBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveBugsRed(
        moveBugsRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveBugsRed2ball(
        moveBugsRed2ball,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveBackBugsRed(
        moveBackBugsRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveDaffyBlue(
        moveDaffyBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveDaffyRed(
        moveDaffyRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveRotateBlue(
        moveRotateBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveRotateRed(
        moveRotateRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_movePreDaffyBlue(
        movePreDaffyBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_movePreDaffyRed(
        movePreDaffyRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    // frc2::SwerveControllerCommand<4> cmd_move_movePreDaffyRed(
    //     movePreDaffyRed2ball,
    //     [&] () { return drivetrain->getPose_m(); },
    //     drivetrain->getKinematics(),
    //     frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
    //     frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
    //     thetaController,
    //     [this] (auto states) { drivetrain->setModuleStates(states); },
    //     {drivetrain}
    // );
    
    frc2::SwerveControllerCommand<4> cmd_move_movePorkyBlue(
        movePorkyBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_movePorkyRed(
        movePorkyRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_movePorkyFromDaffyBlue(
        movePorkyFromDaffyBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

        frc2::SwerveControllerCommand<4> cmd_move_movePorkyFromDaffyRed(
        movePorkyFromDaffyRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveDaffyFromPredaffyBlue(
        moveDaffyFromPredaffyBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveDaffyFromPredaffyRed(
        moveDaffyFromPredaffyRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveShootBlue(
        moveShootBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveShootRed(
        moveShootRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    
    frc2::SwerveControllerCommand<4> cmd_move_moveMarvinBlue(
        moveMarvinBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveMarvinRed(
        moveMarvinRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveAlternateShootBlue(
        moveAlternateShootBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveAlternateShootRed(
        moveAlternateShootRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveStepBackRed(
        moveStepBackRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveStepBackBlue(
        moveStepBackBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );


    frc2::SequentialCommandGroup *shoot3Blue = new frc2::SequentialCommandGroup();
    shoot3Blue->AddCommands
    (cmd_set_odometry,
    //cmd_shooterPrime,
    cmd_shooterAuto,
    cmd_intakeOne,
    cmd_move_moveBugsBlue,
    cmd_move_movePreDaffyBlue,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1.0),
    cmd_nextBall,
    cmd_intakeOne,
    cmd_move_moveDaffyFromPredaffyBlue,
    frc2::WaitCommand((units::second_t).5),
    //cmd_shooterFar,
    frc2::WaitCommand((units::second_t).1),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t).5)
    );

    frc2::SequentialCommandGroup *shoot3Red = new frc2::SequentialCommandGroup();
    shoot3Red->AddCommands
    (cmd_set_odometry,
    //cmd_shooterPrime,
    cmd_shooterAuto,
    cmd_intakeOne,
    cmd_move_moveBugsRed,
    cmd_move_movePreDaffyRed,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1.0),
    cmd_nextBall,
    cmd_intakeOne,
    cmd_move_moveDaffyFromPredaffyRed,
    frc2::WaitCommand((units::second_t).5),
    //cmd_shooterFar,
    frc2::WaitCommand((units::second_t).2),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t).5)
    );


    frc2::SequentialCommandGroup *shoot5Blue = new frc2::SequentialCommandGroup();
    shoot5Blue->AddCommands
    (cmd_set_odometry,
    cmd_turretDisable,
    //cmd_shooterPrime,
    cmd_shooterAuto,
    cmd_intakeOne,
    cmd_move_moveBugsBlue,
    cmd_move_movePreDaffyBlue,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1.0),
    cmd_nextBall,
    cmd_intakeOne,
    cmd_move_moveDaffyFromPredaffyBlue,
    frc2::WaitCommand((units::second_t).5),
    //cmd_shooterFar,
    frc2::WaitCommand((units::second_t).2),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t).5),
    cmd_nextBall,
    cmd_turretDisable,
    cmd_intakeOne,
    cmd_move_movePorkyFromDaffyBlue,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_turretTrack,
    //cmd_intakeDisable,
    cmd_move_moveShootBlue,
    //frc2::WaitCommand((units::second_t).5),
    //cmd_shooterPrime,
    //cmd_shooterAuto,
    //cmd_turretTrack,
    frc2::WaitCommand((units::second_t).75),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1)
    );

    frc2::SequentialCommandGroup *shoot5Red = new frc2::SequentialCommandGroup();
    shoot5Red->AddCommands
    (cmd_set_odometry,
    cmd_printHeading,
    cmd_turretDisable,
    //cmd_shooterPrime,
    cmd_shooterAuto,
    cmd_intakeOne,
    cmd_move_moveBugsRed,
    cmd_move_moveBackBugsRed,
    cmd_printHeading,
    cmd_move_movePreDaffyRed,
    cmd_printHeading,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).25),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t).35),
    cmd_nextBall,
    cmd_intakeOne,
    cmd_move_moveDaffyFromPredaffyRed,
    cmd_printHeading,
    frc2::WaitCommand((units::second_t).25),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t).2),
    cmd_nextBall,
    cmd_turretDisable,
    cmd_intakeOne,
    cmd_move_movePorkyFromDaffyRed,
    cmd_printHeading,
    cmd_move_moveStepBackRed,
    frc2::WaitCommand((units::second_t).65),
    cmd_turretHomeMid,
    cmd_move_moveShootRed,
    cmd_printHeading,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).2),
    cmd_intakeShoot
    );


    frc2::SequentialCommandGroup *shoot2Blue = new frc2::SequentialCommandGroup();
    shoot2Blue->AddCommands
    (cmd_set_alternateOdometry,
    //cmd_shooterPrime,
    cmd_turretHomeMid,
    cmd_shooterAuto,
    cmd_intakeOne,
    cmd_move_moveMarvinBlue,
    cmd_move_moveAlternateShootBlue,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1.0),
    cmd_turretDisable,
    cmd_intakeDisable
    );

    frc2::SequentialCommandGroup *shoot2Red = new frc2::SequentialCommandGroup();
    shoot2Red->AddCommands
    (cmd_set_odometry,
    //cmd_shooterPrime,
    cmd_shooterAuto,
    cmd_intakeOne,
    cmd_move_moveBugsRed,
    cmd_move_movePreDaffyRed,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1.0),
    cmd_set_end_2ball
    // cmd_nextBall,
    // cmd_intakeOne,
    // cmd_move_moveDaffyFromPredaffyRed,
    // frc2::WaitCommand((units::second_t).5),
    // //cmd_shooterFar,
    // frc2::WaitCommand((units::second_t).2),
    // cmd_intakeShoot,
    // frc2::WaitCommand((units::second_t).5)
    );


    frc2::SequentialCommandGroup *shooterTest = new frc2::SequentialCommandGroup();
    shooterTest->AddCommands
    (cmd_set_odometry,
    //cmd_shooterPrime,
    cmd_shooterAuto,
    cmd_intakeOne,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t)3),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1.0),
    cmd_intakeOne
    );

    m_chooser.AddOption("RED 3 ball auto", shoot3Red);
    m_chooser.AddOption("RED 5 ball auto", shoot5Red);
    m_chooser.AddOption("RED 2 ball auto", shoot2Red);

    m_chooser.AddOption("BLUE 3 ball auto", shoot3Blue);
    m_chooser.AddOption("BLUE 5 ball auto", shoot5Blue);
    m_chooser.AddOption("BLUE 2 ball auto", shoot2Blue);

    m_chooser.AddOption("SHOOTER TEST", shooterTest);
    frc::SmartDashboard::PutData(&m_chooser);
}

frc2::Command* ValorAuto::getCurrentAuto() {
    return m_chooser.GetSelected();
}