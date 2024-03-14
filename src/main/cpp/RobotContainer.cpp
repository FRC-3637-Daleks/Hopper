// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <units/math.h>

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

RobotContainer::RobotContainer()
    : m_vision(
          [this](frc::Pose2d pose, units::second_t timestamp,
                 wpi::array<double, 3U> stdDevs) {
            m_swerve.AddVisionPoseEstimate(pose, timestamp, stdDevs);
          },
          [this]() { return m_swerve.GetPose(); },
          Eigen::Matrix<double, 3, 1>{1.0, 1.0, 1.0}) {
  // Initialize all of your commands and subsystems here
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  frc::DataLogManager::LogNetworkTables(true);

  // Log Match Info
  std::string matchType =
      frc::DriverStation::GetMatchType() == frc::DriverStation::MatchType::kNone
          ? ""
          : (frc::DriverStation::GetMatchType() ==
                     frc::DriverStation::MatchType::kElimination
                 ? "Elimination"
                 : (frc::DriverStation::GetMatchType() ==
                            frc::DriverStation::MatchType::kQualification
                        ? "Qualification"
                        : "Practice"));

  std::string alliance =
      (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed
           ? "Red"
           : "Blue");

  frc::DataLogManager::Log(
      fmt::format("Playing {} Match {} at {} as {} alliance\n", matchType,
                  frc::DriverStation::GetMatchNumber(),
                  frc::DriverStation::GetEventName(), alliance));

  // Configure the button bindings
  ConfigureBindings();

  // Configure Dashboard
  ConfigureDashboard();

  // Configure Auton.
  ConfigureAuto();

  frc::DataLogManager::Log(fmt::format("Finished initializing robot."));
}

void RobotContainer::ConfigureBindings() {
  // Configure Swerve Bindings.
  auto fwd = [this]() -> units::meters_per_second_t {
    auto input = frc::ApplyDeadband(
        -m_swerveController.GetRawAxis(OperatorConstants::kForwardAxis),
        OperatorConstants::kDeadband);
    auto squaredInput =
        input * std::abs(input); // square the input while preserving the sign
    return DriveConstants::kMaxTeleopSpeed * squaredInput;
  };

  auto strafe = [this]() -> units::meters_per_second_t {
    auto input = frc::ApplyDeadband(
        -m_swerveController.GetRawAxis(OperatorConstants::kStrafeAxis),
        OperatorConstants::kDeadband);
    auto squaredInput = input * std::abs(input);
    return DriveConstants::kMaxTeleopSpeed * squaredInput;
  };

  auto rot = [this]() -> units::revolutions_per_minute_t {
    auto input = frc::ApplyDeadband(
        -m_swerveController.GetRawAxis(OperatorConstants::kRotationAxis),
        OperatorConstants::kDeadband);
    auto squaredInput = input * std::abs(input);
    return AutoConstants::kMaxAngularSpeed * squaredInput;
  };

  auto checkRed = [this]() -> bool { return m_isRed; };

  auto targetSpeaker = [this]() -> frc::Pose2d {
    return m_isRed ? OperatorConstants::kRedSpeakerPose
                   : OperatorConstants::kBlueSpeakerPose;
  }; // implement live apriltag targeting

  auto targetAMP = [this]() -> frc::Pose2d {
    return m_isRed ? OperatorConstants::kRedAMPPose
                   : OperatorConstants::kBlueAMPPose;
  }; // implement live apriltag targeting

  auto targetStage = [this]() -> frc::Pose2d {
    return m_isRed ? OperatorConstants::kRedStagePose
                   : OperatorConstants::kBlueStagePose;
  }; // implement live apriltag targeting

  auto targetSource = [this]() -> frc::Pose2d {
    return m_isRed ? OperatorConstants::kRedSourcePose
                   : OperatorConstants::kBlueSourcePose;
  };

  constexpr auto targetCenterFarRNote = []() -> frc::Pose2d {
    return OperatorConstants::kCenterFarRNote;
  }; // implement live apriltag targeting

  constexpr auto targetCenterRNote = []() -> frc::Pose2d {
    return OperatorConstants::kCenterRNote;
  }; // implement live apriltag targeting

  constexpr auto targetCenterCNote = []() -> frc::Pose2d {
    return OperatorConstants::kCenterCNote;
  }; // implement live apriltag targeting

  constexpr auto targetCenterLNote = []() -> frc::Pose2d {
    return OperatorConstants::kCenterLNote;
  }; // implement live apriltag targeting

  constexpr auto targetCenterFarLNote = []() -> frc::Pose2d {
    return OperatorConstants::kCenterFarLNote;
  }; // implement live apriltag targeting

  m_swerve.SetDefaultCommand(
      m_swerve.SwerveCommandFieldRelative(fwd, strafe, rot, checkRed));

  m_swerveController.Start().OnTrue(m_swerve.ZeroHeadingCommand());

  m_swerveController.A().WhileTrue(
      m_swerve.ZTargetPoseCommand(targetSource, fwd, strafe, false, checkRed));

  m_swerveController.X().WhileTrue(
      m_swerve.ZTargetPoseCommand(targetSpeaker, fwd, strafe, true, checkRed));

  m_swerveController.B().WhileTrue(
      m_swerve.ZTargetPoseCommand(targetAMP, fwd, strafe, false, checkRed));

  m_swerveController.Y().WhileTrue(
      m_swerve.ZTargetPoseCommand(targetStage, fwd, strafe, false, checkRed));

  m_slowModeTrigger.WhileTrue(
      m_swerve.SwerveSlowCommand(fwd, strafe, rot, checkRed));

  m_swerveController.Back().WhileTrue(m_swerve.SwerveCommand(fwd, strafe, rot));

  m_swerveController.RightBumper().OnTrue(m_intake.ShootOnAMP());

  m_swerveController.LeftBumper().OnTrue(m_intake.OutputToShooter());

  // Configure Shooter Bindings.
  auto flywheel = [this]() -> double {
    return (1.0 - m_copilotController.GetRightTriggerAxis());
  };

  auto pivot = [this]() -> units::degrees_per_second_t {
    return 16_deg_per_s * frc::ApplyDeadband(m_copilotController.GetLeftY(),
                                             OperatorConstants::kDeadband);
  };

  auto calculateDistance = [this]() -> units::meter_t {
    frc::Pose2d RobotPose2d = m_swerve.GetPose();

    // Determine the IDs of the speaker AprilTags based on the alliance color
    int id = m_isRed ? 4 : 7;

    // Get the pose of the speaker AprilTag based on its ID
    frc::Pose3d SpeakerPose = m_aprilTagFieldLayout.GetTagPose(id).value();
    frc::Pose2d SpeakerPose2d = frc::Pose2d{SpeakerPose.X(), SpeakerPose.Y(),
                                            SpeakerPose.Rotation().Angle()};

    // Calculate the horizontal distance between RobotPose and SpeakerPose
    units::meter_t offset =
        RobotPose2d.Translation().Distance(SpeakerPose2d.Translation());
    return offset; // Return the horizontal distance as units::meter_t
  };

  constexpr auto flywheelAutoSpeed = []() { return 0.5; };
  m_shooter.SetDefaultCommand(
      m_shooter.ShooterCommand(flywheel, calculateDistance));

  m_copilotController.Back().WhileTrue(
      m_shooter.ShooterVelocityCommand(flywheel, pivot));

  m_copilotController.RightBumper().WhileTrue(frc2::cmd::Run(
      [this] { m_shooter.SetPivotMotor(m_shooter.ToTalonUnits(43_deg)); },
      {&m_shooter}));

  // Configure Intake Bindings.

  GroundIntakeTrigger.OnTrue(m_intake.IntakeArmIntakeCommand(true));

  AMPIntakeTrigger.OnTrue(m_intake.IntakeArmAMPCommand(true));

  SpeakerIntakeTrigger.OnTrue(m_intake.IntakeArmSpeakerCommand(true));

  AutoIntakeTrigger.OnTrue(m_intake.IntakeRing());

  m_manualIntake.WhileTrue(frc2::cmd::Run(
      [this] {
        if (m_copilotController.GetXButton())
          m_intake.Emergency(1.0);
        else if (m_copilotController.GetYButton())
          m_intake.Emergency(-1.0);
        else
          m_intake.Emergency(0.0);
      },
      {&m_intake}));

  m_copilotController.A().WhileTrue(m_intake.IntakeIn());

  m_copilotController.B().WhileTrue(m_intake.IntakeOut());

  auto climb = [this]() -> double {
    return -frc::ApplyDeadband(m_copilotController.GetRightY(),
                               OperatorConstants::kClimbDeadband);
  };

  m_climb.SetDefaultCommand(m_climb.ClimbCommand(climb));

  pathplanner::ReplanningConfig replanningConfig =
      pathplanner::ReplanningConfig(true, true, .5_m, .125_m);

  constexpr auto alliance = []() -> bool { return false; };

  const pathplanner::HolonomicPathFollowerConfig pathFollowerConfig =
      pathplanner::HolonomicPathFollowerConfig(
          pathplanner::PIDConstants(5.0, 0.0, 0.0), // Translation constants
          pathplanner::PIDConstants(5.0, 0.0, 0.0), // Rotation constants
          ModuleConstants::kPhysicalMaxSpeed,
          DriveConstants::kRadius, // Drive base radius (distance from center to
                                   // furthest module)
          replanningConfig);

  pathplanner::AutoBuilder::configureHolonomic(
      [this]() { return this->m_swerve.GetPose(); },
      [this](frc::Pose2d pose) { this->m_swerve.ResetOdometry(pose); },
      [this]() { return this->m_swerve.GetSpeed(); },
      [this](frc::ChassisSpeeds speed) {
        this->m_swerve.Drive(speed.vx, speed.vy, speed.omega, false, m_isRed);
      },
      pathFollowerConfig,
      [this]() { return m_isRed; }, // replace later, just a placeholder
      (&m_swerve));
  // m_swerve.SetDefaultCommand(m_swerve.SwerveCommand(fwd, strafe, rot));
  //  m_swerveController.Button(OperatorConstants::kFieldRelativeButton)
  //     .WhileTrue(m_swerve.SwerveCommandFieldRelative(fwd, strafe, rot));

  pathplanner::NamedCommands::registerCommand(
      "ShootCommand",
      m_shooter.ShooterCommand(flywheelAutoSpeed, calculateDistance)
          .WithName("ShootCommand")); // this aint right but ill change it at
                                      // some point
  pathplanner::NamedCommands::registerCommand(
      "ShootAmp", m_intake.ShootOnAMP().WithName("ShootAMP"));
  // need to find out what the output command is, how all that stuff works and
  // implement here also need to see if the Shoot Command will work as it is
  // currently configured
  pathplanner::NamedCommands::registerCommand(
      "IntakeRing", m_intake.IntakeRing().WithName("IntakeRing"));
  pathplanner::NamedCommands::registerCommand(
      "OutputToShooter",
      frc2::cmd::Sequence(
          m_swerve
              .ZTargetPoseCommand(targetSpeaker, fwd, strafe, true, alliance)
              .WithTimeout(1_s),
          m_intake.OutputToShooter().WithName("OutputToShooter")));
  pathplanner::NamedCommands::registerCommand(
      "zTargetingCenterNoteFarR",
      m_swerve
          .ZTargetPoseCommand(targetCenterFarRNote, fwd, strafe, false,
                              alliance)
          .WithTimeout(1_s));
  pathplanner::NamedCommands::registerCommand(
      "zTargetingCenterNoteR",
      m_swerve
          .ZTargetPoseCommand(targetCenterRNote, fwd, strafe, false, alliance)
          .WithTimeout(1_s));
  pathplanner::NamedCommands::registerCommand(
      "zTargetingCenterNoteC",
      m_swerve
          .ZTargetPoseCommand(targetCenterCNote, fwd, strafe, false, alliance)
          .WithTimeout(1_s));
  pathplanner::NamedCommands::registerCommand(
      "zTargetingCenterNoteL",
      m_swerve
          .ZTargetPoseCommand(targetCenterLNote, fwd, strafe, false, alliance)
          .WithTimeout(1_s));
  pathplanner::NamedCommands::registerCommand(
      "zTargetingCenterNoteFarL",
      m_swerve
          .ZTargetPoseCommand(targetCenterFarLNote, fwd, strafe, false,
                              alliance)
          .WithTimeout(1_s));

  pathplanner::PathConstraints constraints = pathplanner::PathConstraints(
      AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration,
      AutoConstants::kMaxAngularSpeed, AutoConstants::kMaxAngularAcceleration);

  auto BlueSourcePath = pathplanner::AutoBuilder::pathfindToPose(
      OperatorConstants::kBlueSourcePickUp, constraints, 0_mps, 0_m);
  auto RedSourcePath = pathplanner::AutoBuilder::pathfindToPose(
      OperatorConstants::kRedSourcePickUp, constraints, 0_mps, 0_m);
  auto BlueAmpShotPath = pathplanner::AutoBuilder::pathfindToPose(
      OperatorConstants::kBlueAmpShot, constraints, 0_mps, 0_m);
  auto RedAmpShotPath = pathplanner::AutoBuilder::pathfindToPose(
      OperatorConstants::kRedAmpShot, constraints, 0_mps, 0_m);

  m_SourcePath = frc2::cmd::Either(std::move(RedSourcePath),
                                   std::move(BlueSourcePath), checkRed);
  m_AmpShotPath = frc2::cmd::Either(std::move(RedAmpShotPath),
                                    std::move(BlueAmpShotPath), checkRed);
  m_left3NoteAuto = pathplanner::PathPlannerAuto("Left 3 Note").ToPtr();
  m_right3NoteAuto = pathplanner::PathPlannerAuto("Right 3 Note").ToPtr();
  m_center3NoteAuto = pathplanner::PathPlannerAuto("Center 3 Note").ToPtr();

  m_left2NoteAuto = pathplanner::PathPlannerAuto("Left 2 Note").ToPtr();
  m_right2NoteAuto = pathplanner::PathPlannerAuto("Right 2 Note").ToPtr();
  m_center2NoteAuto = pathplanner::PathPlannerAuto("Center 2 Note").ToPtr();

  m_leftCenterOnlyAuto =
      pathplanner::PathPlannerAuto("Left 3 Note Mid Only").ToPtr();
  m_rightCenterOnlyAuto =
      pathplanner::PathPlannerAuto("Right 3 Note Mid Only").ToPtr();
  m_centerRightCenterOnlyAuto =
      pathplanner::PathPlannerAuto("Center-Right 3 Note Mid Only").ToPtr();
  m_centerLeftCenterOnlyAuto =
      pathplanner::PathPlannerAuto("Center-Left 3 Note Mid Only").ToPtr();

  m_getOutRight = pathplanner::PathPlannerAuto("Get Out Right").ToPtr();

  SourcePathTrigger.WhileTrue(m_SourcePath.get());
  AmpPathTrigger.WhileTrue(m_AmpShotPath.get());

  m_chooser.SetDefaultOption("Left (Amp-Side) Subwoofer 3 Note Auto",
                             m_left3NoteAuto.get());
  m_chooser.AddOption("Right (Source-Side) Subwoofer 3 Note Auto",
                      m_right3NoteAuto.get());
  m_chooser.AddOption("Center Subwoofer 3 Note Auto", m_center3NoteAuto.get());

  m_chooser.AddOption("Center Subwoofer 2 Note Auto", m_center2NoteAuto.get());
  m_chooser.AddOption("Right (Source-Side) Subwoofer 2 Note Auto",
                      m_right2NoteAuto.get());
  m_chooser.AddOption("Left (AMP-Side) Subwoofer 2 Note Auto",
                      m_left2NoteAuto.get());

  m_chooser.AddOption("CenterRight Mid Only 3 Note Auto",
                      m_centerRightCenterOnlyAuto.get());
  m_chooser.AddOption("CenterLeft Mid Only 3 Note Auto",
                      m_centerLeftCenterOnlyAuto.get());
  m_chooser.AddOption("Right Mid Only 3 Note Auto",
                      m_rightCenterOnlyAuto.get());
  m_chooser.AddOption("Left Center Only 3 Note Auto",
                      m_leftCenterOnlyAuto.get());

  m_chooser.AddOption("Get out right Side", m_getOutRight.get());

  frc::SmartDashboard::PutData(&m_chooser);
}

void RobotContainer::ConfigureDashboard() {
  m_intake.InitVisualization(&m_mech_sideview);
  m_shooter.InitVisualization(&m_mech_sideview);

  frc::SmartDashboard::PutData("Mechanisms", &m_mech_sideview);
  frc::SmartDashboard::PutData("Intake", &m_intake);
  frc::SmartDashboard::PutData("Shooter", &m_shooter);
  frc::SmartDashboard::PutData("Drivebase", &m_swerve);
}

void RobotContainer::ConfigureAuto() {
  pathplanner::PathPlannerLogging::setLogActivePathCallback(
      [this](auto &&activePath) {
        m_swerve.GetField().GetObject("Hopper")->SetPoses(activePath);
      });
}

frc2::Command *RobotContainer::GetAutonomousCommand() {
  return m_chooser.GetSelected();
}

frc2::CommandPtr RobotContainer::GetDisabledCommand() {
  return frc2::cmd::None();
}