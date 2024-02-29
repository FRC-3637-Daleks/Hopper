// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <units/math.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>


RobotContainer::RobotContainer() : m_vision([this](frc::Pose2d pose, units::second_t timestamp,
                                  wpi::array<double, 3U> stdDevs){
                                    m_swerve.AddVisionPoseEstimate(pose, timestamp, stdDevs);
                                  }, [this](){
                                    return m_swerve.GetPose();
                                  }, Eigen::Matrix<double, 3, 1>{1.0, 1.0, 1.0}){
                                   
  fmt::println("made it to robot container");
  // Initialize all of your commands and subsystems here
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  frc::DataLogManager::LogNetworkTables(true);

  // Configure the button bindings
  ConfigureBindings();

  // Configure Dashboard
  ConfigureDashboard();

  // Configure Auton.
  ConfigureAuto();
}

void RobotContainer::ConfigureBindings() {
  // Configure Swerve Bindings.
  auto fwd = [this]() -> units::meters_per_second_t {
    auto input = frc::ApplyDeadband(-m_swerveController.GetRawAxis(OperatorConstants::kForwardAxis), OperatorConstants::kDeadband);
    auto squaredInput = input * std::abs(input); // square the input while preserving the sign
    return DriveConstants::kMaxTeleopSpeed * squaredInput;
  };

  auto strafe = [this]() -> units::meters_per_second_t {
    auto input = frc::ApplyDeadband(-m_swerveController.GetRawAxis(OperatorConstants::kStrafeAxis), OperatorConstants::kDeadband);
    auto squaredInput = input * std::abs(input); 
    return DriveConstants::kMaxTeleopSpeed * squaredInput;
  };

  auto rot = [this]() -> units::revolutions_per_minute_t {
    auto input = frc::ApplyDeadband(-m_swerveController.GetRawAxis(OperatorConstants::kRotationAxis), OperatorConstants::kDeadband);
    auto squaredInput = input * std::abs(input); 
    return AutoConstants::kMaxAngularSpeed * squaredInput;
  };

  auto checkRed = [this] () -> bool { return m_isRed; };

  auto targetSpeaker = [this] () -> frc::Pose2d { return m_isRed ? OperatorConstants::kRedSpeakerPose : OperatorConstants::kBlueSpeakerPose; }; //implement live apriltag targeting
  auto targetAMP = [this] () -> frc::Pose2d { return m_isRed ? OperatorConstants::kRedAMPPose : OperatorConstants::kBlueAMPPose; }; //implement live apriltag targeting
  auto targetStage = [this] () -> frc::Pose2d { return m_isRed ? OperatorConstants::kRedStagePose : OperatorConstants::kBlueStagePose; }; //implement live apriltag targeting
  auto targetSource = [this] () -> frc::Pose2d { return m_isRed ? OperatorConstants::kRedSourcePose : OperatorConstants::kBlueSourcePose; }; //implement live apriltag targeting

  constexpr auto SubWoofer = [] () -> frc::Pose2d { return {-2_m, 0_m, 0_rad}; };

  m_swerve.SetDefaultCommand(m_swerve.SwerveCommandFieldRelative(fwd, strafe, rot, checkRed));

  m_swerveController.Start()
      .OnTrue(m_swerve.ZeroHeadingCommand());

  m_swerveController.A()
    .WhileTrue(m_swerve.ZTargetPoseCommand(targetSource, fwd, strafe, false));

  m_swerveController.B()
    .WhileTrue(m_swerve.ZTargetPoseCommand(targetSpeaker, fwd, strafe, true));

  m_swerveController.X()
    .WhileTrue(m_swerve.ZTargetPoseCommand(targetAMP, fwd, strafe, false));

  m_swerveController.Y()
    .WhileTrue(m_swerve.ZTargetPoseCommand(targetStage, fwd, strafe, false));

  m_swerveController.LeftStick()
      .WhileTrue(m_swerve.SwerveSlowCommand(fwd,strafe,rot, checkRed));
  
  m_swerveController.Back()
      .WhileTrue(m_swerve.SwerveCommand(fwd, strafe, rot));
  
  m_swerveController.RightBumper()
      //.WhileTrue(m_swerve.SwerveCommand(fwd, strafe, rot));
      .OnTrue(m_intake.ShootOnAMP());

  m_swerveController.LeftBumper()
      .OnTrue(m_intake.OutputToShooter());
  
  //m_swerveController.X().WhileTrue(m_swerve.ZeroAbsEncodersCommand());
  // m_swerveController.LeftBumper().WhileTrue(m_swerve.ConfigAbsEncoderCommand());
      
  //Configure Shooter Bindings.
  auto flywheel = [this] () -> double {
    return (1.0 - m_copilotController.GetRightTriggerAxis());
  };

  auto pivot = [this] () -> units::degrees_per_second_t {
    return 16_deg_per_s * frc::ApplyDeadband(m_copilotController.GetLeftY(), OperatorConstants::kDeadband);
  };
  
  auto calculateDistance = [this]() -> units::meter_t {
      frc::Pose2d RobotPose2d = m_swerve.GetPose();
      
      // Determine the IDs of the speaker AprilTags based on the alliance color
      int id = m_isRed ? 4 : 7;

      // Get the pose of the speaker AprilTag based on its ID
      frc::Pose3d SpeakerPose = m_aprilTagFieldLayout.GetTagPose(id).value();
      frc::Pose2d SpeakerPose2d = frc::Pose2d{SpeakerPose.X(), SpeakerPose.Y(), SpeakerPose.Rotation().Angle()};
      // auto it1 = m_aprilTagFieldLayout.GetTagPose(id);
      // if (it1.has_value()) {
      //     SpeakerPose = it1.value();
      // } else {
      //     // Handle case where neither tag is found
      //     SpeakerPose = frc::Pose3d();
      // }
      
      units::meter_t z = 1.5_ft; // estimation of shooter height 
    
      // frc::SmartDashboard::PutData(RobotPose3d);
      // Calculate the horizontal distance between RobotPose and SpeakerPose
      units::meter_t offset = RobotPose2d.Translation().Distance(SpeakerPose2d.Translation());
      return offset; //Return the horizontal distance as units::meter_t
  };


  constexpr auto flywheelAutoSpeed = []()
  {
   return 0.5;
  };
  m_shooter.SetDefaultCommand(m_shooter.ShooterCommand(flywheel, calculateDistance));

  m_copilotController.Back()
    .WhileTrue(m_shooter.ShooterVelocityCommand(flywheel, pivot));

  // Configure Intake Bindings.
  auto position = [this]() -> int {
    return m_copilotController.GetPOV();
  };

  // m_intake.SetDefaultCommand(
  //   frc2::cmd::Select<int>(
  //     position,
  //     std::pair<int, frc2::CommandPtr>{-1, m_intake.IdleIntakeCommand()},
  //     std::pair<int, frc2::CommandPtr>{OperatorConstants::kIntakeGroundPOV, m_intake.IntakeArmIntakeCommand(false)},
  //     std::pair<int, frc2::CommandPtr>{OperatorConstants::kIntakeAMPPOV, m_intake.IntakeArmAMPCommand(false)},
  //     std::pair<int, frc2::CommandPtr>{OperatorConstants::kIntakeShooterPOV, m_intake.IntakeArmSpeakerCommand(false)},
  //     std::pair<int, frc2::CommandPtr>{OperatorConstants::kAutoIntake, m_intake.IntakeRing()}
  //     )
  // );

  frc2::Trigger IdleIntakeTrigger([this] { return m_copilotController.GetPOV() == -1; });
  frc2::Trigger GroundIntakeTrigger([this] { return m_copilotController.GetPOV() == OperatorConstants::kIntakeGroundPOV; });
  frc2::Trigger AMPIntakeTrigger([this] { return m_copilotController.GetPOV() == OperatorConstants::kIntakeAMPPOV; });
  frc2::Trigger SpeakerIntakeTrigger([this] { return m_copilotController.GetPOV() == OperatorConstants::kIntakeShooterPOV; });
  frc2::Trigger AutoIntakeTrigger([this] { return m_copilotController.GetPOV() == OperatorConstants::kAutoIntake; });
  
  GroundIntakeTrigger
    .OnTrue(m_intake.IntakeArmIntakeCommand(true));

  AMPIntakeTrigger
    .OnTrue(m_intake.IntakeArmAMPCommand(true));
  
  SpeakerIntakeTrigger
    .OnTrue(m_intake.IntakeArmSpeakerCommand(true));

  AutoIntakeTrigger
    .OnTrue(m_intake.IntakeRing());


  m_copilotController.A()
    .WhileTrue(m_intake.IntakeIn());
    
  m_copilotController.B()
    .WhileTrue(m_intake.IntakeOut());

  auto climb = [this] () -> double { return -frc::ApplyDeadband(m_copilotController.GetRightY(), OperatorConstants::kDeadband); };

  m_climb.SetDefaultCommand(m_climb.ClimbCommand(climb)); 





  const pathplanner::HolonomicPathFollowerConfig pathFollowerConfig = pathplanner::HolonomicPathFollowerConfig(
    pathplanner::PIDConstants(1.0, 0.0, 0.0), // Translation constants 
    pathplanner::PIDConstants(1.0, 0.0, 0.0), // Rotation constants 
    AutoConstants::kMaxSpeed,
    DriveConstants::kRadius, // Drive base radius (distance from center to furthest module) 
    pathplanner::ReplanningConfig());


    pathplanner::AutoBuilder::configureHolonomic(
          [this](){return this->m_swerve.GetPose();},
          [this](frc::Pose2d pose){this->m_swerve.ResetOdometry(pose);},
          [this](){return this->m_swerve.GetSpeed();},
          [this](frc::ChassisSpeeds speed){this->m_swerve.Drive(
          speed.vx, speed.vy, speed.omega, false, false);},
          pathFollowerConfig,
          [this](){  
            return m_isRed;            
          }, //replace later, just a placeholder
          (&m_swerve)
    );
  // m_swerve.SetDefaultCommand(m_swerve.SwerveCommand(fwd, strafe, rot));
  //  m_swerveController.Button(OperatorConstants::kFieldRelativeButton)
  //     .WhileTrue(m_swerve.SwerveCommandFieldRelative(fwd, strafe, rot));
      

      pathplanner::NamedCommands::registerCommand("ShootCommand", m_shooter.ShooterCommand(flywheelAutoSpeed, calculateDistance).WithName("ShootCommand")); //this aint right but ill change it at some point
      pathplanner::NamedCommands::registerCommand("ShootAmp", m_intake.ShootOnAMP().WithName("ShootAMP"));
      //need to find out what the output command is, how all that stuff works and implement here
      //also need to see if the Shoot Command will work as it is currently configured
      pathplanner::NamedCommands::registerCommand("IntakeRing", m_intake.IntakeRing().WithName("IntakeRing"));
      pathplanner::NamedCommands::registerCommand("OutputToShooter", m_intake.OutputToShooter().WithName("OutputToShooter"));
      pathplanner::NamedCommands::registerCommand("zTargetingSpeaker", m_swerve.ZTargetPoseCommand(targetSpeaker, fwd, strafe, true).WithTimeout(2_s).WithName("zTargetSpeaker"));
      pathplanner::NamedCommands::registerCommand("zTargetingAmp", m_swerve.ZTargetPoseCommand(targetAMP, fwd, strafe, false).WithName("zTargetAmp"));
      pathplanner::NamedCommands::registerCommand("zTargetingSource", m_swerve.ZTargetPoseCommand(targetSource, fwd, strafe, false));
      pathplanner::NamedCommands::registerCommand("zTargetingStage", m_swerve.ZTargetPoseCommand(targetStage, fwd, strafe, false));

      m_rightSubAuto = pathplanner::PathPlannerAuto("RightSubStart").ToPtr();
      m_centerSubAuto = pathplanner::PathPlannerAuto("CenterSubStart").ToPtr();
      m_leftSubAuto = pathplanner::PathPlannerAuto("LeftSubStart").ToPtr();
      m_chooser.SetDefaultOption("Left Subwoofer Auto", m_leftSubAuto.get());
      m_chooser.AddOption("Right Subwoofer Auto", m_rightSubAuto.get());
      m_chooser.AddOption("Center Subwoofer Auto", m_centerSubAuto.get());

      frc::SmartDashboard::PutData(&m_chooser);
}


void RobotContainer::ConfigureDashboard()
{
  m_intake.InitVisualization(&m_mech_sideview);
  m_shooter.InitVisualization(&m_mech_sideview);

  frc::SmartDashboard::PutData("Mechanisms", &m_mech_sideview);
  frc::SmartDashboard::PutData("Intake", &m_intake);
  frc::SmartDashboard::PutData("Shooter", &m_shooter);
  frc::SmartDashboard::PutData("Drivebase", &m_swerve);
}

void RobotContainer::ConfigureAuto()
{
      pathplanner::PathPlannerLogging::setLogActivePathCallback([this](auto&& activePath) {
        m_swerve.GetField().GetObject("Auto Path")->SetPoses(activePath);
      });
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  // You can ignore this for now.
  //return autos::ExampleAuto(&m_subsystem);
  return pathplanner::PathPlannerAuto("RightSubStart").ToPtr();
  return frc2::cmd::Idle();
}

frc2::CommandPtr RobotContainer::GetDisabledCommand(){
  // return m_swerve.CoastModeCommand(true).IgnoringDisable(true);
  return frc2::cmd::None();
}
