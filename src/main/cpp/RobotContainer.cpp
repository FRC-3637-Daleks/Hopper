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
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

RobotContainer::RobotContainer()
    : m_vision(
          [this](frc::Pose2d pose, units::second_t timestamp,
                 wpi::array<double, 3U> stdDevs) {
            m_swerve.AddVisionPoseEstimate(pose, timestamp, stdDevs);
          },
          [this]() { return m_swerve.GetPose(); },
          Eigen::Matrix<double, 3, 1>{1.0, 1.0, 1.0},
          [this] { return m_swerve.GetSimulatedGroundTruth(); }) {

  fmt::println("made it to robot container");
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

void RobotContainer::ControllerRumble() {
  m_swerveController.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 1.0);
  m_swerveController.SetRumble(frc::GenericHID::RumbleType::kRightRumble, 1.0);
  frc::Wait(300_ms);
  m_swerveController.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.0);
  m_swerveController.SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0.0);
}

void RobotContainer::ConfigureBindings() {
  RumbleForIntakeTrigger.OnTrue(frc2::cmd::RunOnce([this] { ControllerRumble(); }));
  RumbleForOutakeTrigger.OnTrue(frc2::cmd::RunOnce([this] { ControllerRumble(); }));

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

  // Constantly updating for alliance checks.
  auto checkRed = [this]() -> bool { return m_isRed; };

  // Z-Target locations

  auto targetSpeaker = [this]() -> frc::Pose2d {
    return m_isRed ? OperatorConstants::kRedSpeakerPose
                   : OperatorConstants::kBlueSpeakerPose;
  };

  auto targetAMP = [this]() -> frc::Pose2d {
    return m_isRed ? OperatorConstants::kRedAMPPose
                   : OperatorConstants::kBlueAMPPose;
  };

  auto targetStage = [this]() -> frc::Pose2d {
    return m_isRed ? OperatorConstants::kRedStagePose
                   : OperatorConstants::kBlueStagePose;
  };

  auto targetSource = [this]() -> frc::Pose2d {
    return m_isRed ? OperatorConstants::kRedSourcePose
                   : OperatorConstants::kBlueSourcePose;
  };

  constexpr auto targetMidFarRNote = []() -> frc::Pose2d {
    return OperatorConstants::kMidFarRNote;
  };

  constexpr auto targetMidRNote = []() -> frc::Pose2d {
    return OperatorConstants::kMidRNote;
  };

  constexpr auto targetMidCNote = []() -> frc::Pose2d {
    return OperatorConstants::kMidCNote;
  };

  constexpr auto targetMidLNote = []() -> frc::Pose2d {
    return OperatorConstants::kMidLNote;
  };

  constexpr auto targetMidFarLNote = []() -> frc::Pose2d {
    return OperatorConstants::kMidFarLNote;
  };

  m_swerve.SetDefaultCommand(
      m_swerve.SwerveCommandFieldRelative(fwd, strafe, rot, checkRed));

  m_swerveController.Start().OnTrue(m_swerve.ZeroHeadingCommand());

  m_swerveController.A().WhileTrue(
      m_swerve.ZTargetPoseCommand(targetSource, fwd, strafe, false, checkRed));

  m_swerveController.X().WhileTrue(
      m_swerve.ZTargetPoseCommand(targetSpeaker, fwd, strafe, true, checkRed));

  m_swerveController.B().WhileTrue(
      m_swerve.ZTargetPoseCommand(targetAMP, fwd, strafe, true, checkRed));

  m_swerveController.Y().WhileTrue(
      m_swerve.ZTargetPoseCommand(targetStage, fwd, strafe, false, checkRed));

  m_slowModeTrigger.WhileTrue(
      m_swerve.SwerveSlowCommand(fwd, strafe, rot, checkRed));

  m_swerveController.Back().WhileTrue(m_swerve.SwerveCommand(fwd, strafe, rot));

  m_swerveController.RightBumper().OnTrue(m_intake.ShootOnAMPRetract());

  m_swerveController.LeftBumper().OnTrue(m_intake.OutputToShooter());

  constexpr auto one_meter = []() -> units::meters_per_second_t {
    return 1_mps;
  };

  constexpr auto neg_one_meter = []() -> units::meters_per_second_t {
    return -1_mps;
  };

  // Precise driving commands.

  DriveFwdTrigger.WhileTrue(
      m_swerve.SwerveCommandFieldRelative(one_meter, strafe, rot, checkRed));

  DriveStrafeLeftTrigger.WhileTrue(
      m_swerve.SwerveCommandFieldRelative(fwd, one_meter, rot, checkRed));

  DriveRevTrigger.WhileTrue(m_swerve.SwerveCommandFieldRelative(
      neg_one_meter, strafe, rot, checkRed));

  DriveStrafeRightTrigger.WhileTrue(
      m_swerve.SwerveCommandFieldRelative(fwd, neg_one_meter, rot, checkRed));

  // Configure Shooter Bindings.
  auto flywheel = [this]() -> double {
    return (1.0 - m_copilotController.GetRightTriggerAxis());
  };

  auto pivot = [this]() -> units::degrees_per_second_t {
    return 16_deg_per_s * frc::ApplyDeadband(m_copilotController.GetLeftY(),
                                             OperatorConstants::kDeadband);
  };

  auto calculateSpeakerDistance = [this]() -> units::meter_t {
    frc::Pose2d RobotPose2d = m_swerve.GetPose();

    // Determine the IDs of the speaker AprilTags based on the alliance color
    int speakerID = m_isRed ? 4 : 7;

    // Get the pose of the speaker AprilTag based on its ID
    frc::Pose3d SpeakerPose =
        m_aprilTagFieldLayout.GetTagPose(speakerID).value();
    frc::Pose2d SpeakerPose2d = frc::Pose2d{SpeakerPose.X(), SpeakerPose.Y(),
                                            SpeakerPose.Rotation().Angle()};

    // Calculate the horizontal distance between RobotPose and SpeakerPose
    units::meter_t offset =
        RobotPose2d.Translation().Distance(SpeakerPose2d.Translation());
    return offset; // Return the horizontal distance as units::meter_t
  };

  auto calculateAmpDistance = [this]() -> units::meter_t {
    frc::Pose2d RobotPose2d = m_swerve.GetPose();

    // Determine the IDs of the speaker AprilTags based on the alliance color
    int ampID = m_isRed ? 5 : 6;

    // Get the pose of the speaker AprilTag based on its ID
    frc::Pose3d AmpPose = m_aprilTagFieldLayout.GetTagPose(ampID).value();
    frc::Pose2d AmpPose2d =
        frc::Pose2d{AmpPose.X(), AmpPose.Y(), AmpPose.Rotation().Angle()};

    // Calculate the horizontal distance between RobotPose and SpeakerPose
    units::meter_t offset =
        RobotPose2d.Translation().Distance(AmpPose2d.Translation());
    return offset; // Return the horizontal distance as units::meter_t
  };

  m_shooter.SetDefaultCommand(
      m_shooter.ShooterCommand(flywheel, calculateSpeakerDistance));

  m_copilotController.LeftStick().ToggleOnTrue(
      m_shooter.ShooterVelocityCommand(flywheel, pivot));

  m_copilotController.RightBumper().WhileTrue(m_shooter.SubwooferCommand());

  m_copilotController.LeftBumper().WhileTrue(m_shooter.AmpShot());

  // Configure Intake Bindings.

  GroundIntakeTrigger.OnTrue(m_intake.IntakeArmIntakeCommand(true));

  AMPIntakeTrigger.OnTrue(m_intake.IntakeFromPlayerStation());

  SpeakerIntakeTrigger.OnTrue(m_intake.IntakeArmSpeakerCommand(true));

  AutoIntakeTrigger.OnTrue(m_intake.IntakeRing());

  // Manual intake using percent out.

  m_copilotController.Start().ToggleOnTrue(frc2::cmd::Run(
      [this] {
        if (m_copilotController.GetXButton())
          m_intake.Emergency(1.0);
        else if (m_copilotController.GetYButton())
          m_intake.Emergency(-1.0);
        else
          m_intake.Emergency(0.0);
      },
      {&m_intake}));

  m_passMode.WhileTrue(m_shooter.PassModeCommand());

  // Manual Intake In/Out.
  m_copilotController.A().WhileTrue(m_intake.IntakeIn());

  m_copilotController.B().WhileTrue(m_intake.IntakeOut());

  constexpr auto flywheelOff = []() { return 0.0; };

  PitReset.OnTrue(frc2::cmd::Parallel(
      m_shooter.PivotAngleCommand([]() { return 80_deg; }),
      m_shooter.FlywheelCommand(flywheelOff), m_climb.RetractClimb(),
      m_intake.IntakeArmSpeakerCommand()));

  // Configure climb bindings.

  auto climb = [this]() -> double {
    return -frc::ApplyDeadband(m_copilotController.GetRightY(),
                               OperatorConstants::kClimbDeadband);
  };

  m_climb.SetDefaultCommand(m_climb.ClimbCommand(climb));

  // Configure PathPlanner.

  /**
   * If the robot falls off the course of a path, replanning may be required.
   */
  pathplanner::ReplanningConfig replanningConfig =
      pathplanner::ReplanningConfig(true, true, 1_m, .25_m);

  constexpr auto alliance = []() -> bool { return false; };

  /**
   * Apply the swerve drive configurations.
   */
  const pathplanner::HolonomicPathFollowerConfig pathFollowerConfig =
      pathplanner::HolonomicPathFollowerConfig(
          pathplanner::PIDConstants(10.0, 0.0, 0.0), // Translation constants
          pathplanner::PIDConstants(15.0, 0.0, 0.0), // Rotation constants
          ModuleConstants::kPhysicalMaxSpeed,
          DriveConstants::kRadius, // Drive base radius (distance from center to
                                   // furthest module)
          replanningConfig);

  pathplanner::AutoBuilder::configureHolonomic(
      [this]() { return this->m_swerve.GetPose(); },
      [this](frc::Pose2d pose) { this->m_swerve.ResetOdometry(pose); },
      [this]() { return this->m_swerve.GetSpeed(); },
      [this](frc::ChassisSpeeds speed) {
        auto rotationOverride = GetRotationTargetOverride();
        if (rotationOverride.has_value())
          this->m_swerve.OverrideAngle(rotationOverride.value(), speed.vx,
                                       speed.vy, m_isRed);
        else
          this->m_swerve.Drive(speed.vx, speed.vy, speed.omega, false, m_isRed);
      },
      pathFollowerConfig,
      [this]() { return m_isRed; }, // replace later, just a placeholder
      (&m_swerve));

  pathplanner::PPHolonomicDriveController::setRotationTargetOverride(
      [this] { return GetRotationTargetOverride(); });

  pathplanner::PathConstraints constraints = pathplanner::PathConstraints(
      AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration,
      AutoConstants::kMaxAngularSpeed, AutoConstants::kMaxAngularAcceleration);

  // Register named commands for use in auton.

  pathplanner::NamedCommands::registerCommand(
      "ShootAmp", m_intake.ShootOnAMPRetract().WithName("ShootAMP"));

  pathplanner::NamedCommands::registerCommand(
      "IntakeRing", m_intake.IntakeRing().WithName("IntakeRing"));

  pathplanner::NamedCommands::registerCommand(
      "OutputToShooterZTarget",
      frc2::cmd::Sequence(
          m_swerve
              .ZTargetPoseCommand(targetSpeaker, fwd, strafe, true, alliance)
              .WithTimeout(1_s),
          m_intake.OutputToShooter().WithTimeout(1_s).WithName(
              "OutputToShooterZTarget")));

  pathplanner::NamedCommands::registerCommand(
      "OutputToShooter",
      m_intake.OutputToShooter().WithName("OutputToShooter"));

  pathplanner::NamedCommands::registerCommand(
      "ShootAmp", frc2::cmd::None().WithName("ShootAmp"));

  pathplanner::NamedCommands::registerCommand(
      "zTargetingMidNoteFarR",
      m_swerve
          .ZTargetPoseCommand(targetMidFarRNote, fwd, strafe, false, alliance)
          .WithTimeout(1_s));

  pathplanner::NamedCommands::registerCommand(
      "zTargetingMidNoteR",
      m_swerve.ZTargetPoseCommand(targetMidRNote, fwd, strafe, false, alliance)
          .WithTimeout(1_s));

  pathplanner::NamedCommands::registerCommand(
      "zTargetingMidNoteC",
      m_swerve.ZTargetPoseCommand(targetMidCNote, fwd, strafe, false, alliance)
          .WithTimeout(1_s));

  pathplanner::NamedCommands::registerCommand(
      "zTargetingMidNoteL",
      m_swerve.ZTargetPoseCommand(targetMidLNote, fwd, strafe, false, alliance)
          .WithTimeout(1_s));

  pathplanner::NamedCommands::registerCommand(
      "zTargetingMidNoteFarL",
      m_swerve
          .ZTargetPoseCommand(targetMidFarLNote, fwd, strafe, false, alliance)
          .WithTimeout(1_s));

  pathplanner::NamedCommands::registerCommand(
      "StraightenRobot",
      frc2::cmd::Either(m_swerve.TurnToAngleCommand(180_deg),
                        m_swerve.TurnToAngleCommand(0_deg), checkRed));

  // Special pathfinding configurations.

  auto BlueSourcePath = pathplanner::AutoBuilder::pathfindToPose(
      OperatorConstants::kBlueSourcePickUp, constraints, 0_mps, 0_m);

  auto RedSourcePath = pathplanner::AutoBuilder::pathfindToPose(
      OperatorConstants::kRedSourcePickUp, constraints, 0_mps, 0_m);

  auto BlueAmpShotPath = pathplanner::AutoBuilder::pathfindToPose(
      OperatorConstants::kBlueAmpShot, constraints, 0_mps, 0_m);

  auto RedAmpShotPath = pathplanner::AutoBuilder::pathfindToPose(
      OperatorConstants::kRedAmpShot, constraints, 0_mps, 0_m);

  auto BlueCenterSubPath = pathplanner::AutoBuilder::pathfindToPose(
      OperatorConstants::kBlueCenterSub, constraints, 0_mps, 0_m);

  auto RedCenterSubPath = pathplanner::AutoBuilder::pathfindToPose(
      OperatorConstants::kRedCenterSub, constraints, 0_mps, 0_m);

  m_SourcePath = frc2::cmd::Either(std::move(RedSourcePath),
                                   std::move(BlueSourcePath), checkRed);

  m_AmpShotPath = frc2::cmd::Either(std::move(RedAmpShotPath),
                                    std::move(BlueAmpShotPath), checkRed);

  m_CenterSubPath = frc2::cmd::Either(std::move(RedCenterSubPath),
                                      std::move(BlueCenterSubPath), checkRed);

  // Load autons.

  m_defaultAuto = pathplanner::PathPlannerAuto("Default Auto").ToPtr();

  m_AmpSide3NoteAuto = pathplanner::PathPlannerAuto("AmpSide 3 Note").ToPtr();
  m_SourceSide3NoteAuto =
      pathplanner::PathPlannerAuto("SourceSide 3 Note").ToPtr();
  m_center3NoteAuto = pathplanner::PathPlannerAuto("Center 3 Note").ToPtr();

  m_AmpSide2NoteAuto = pathplanner::PathPlannerAuto("AmpSide 2 Note").ToPtr();
  m_SourceSide2NoteAuto =
      pathplanner::PathPlannerAuto("SourceSide 2 Note").ToPtr();
  m_center2NoteAuto = pathplanner::PathPlannerAuto("Center 2 Note").ToPtr();

  m_AmpSideMidOnlyAuto =
      pathplanner::PathPlannerAuto("AmpSide 3 Note Mid Only").ToPtr();
  m_SourceSideMidOnlyAuto =
      pathplanner::PathPlannerAuto("SourceSide 3 Note Mid Only").ToPtr();
  m_SourceSideMidInnerOnlyAuto =
      pathplanner::PathPlannerAuto("SourceSide 3 Note Mid Only Inner First")
          .ToPtr();
  m_centerSourceSideMidOnlyAuto =
      pathplanner::PathPlannerAuto("Center-SourceSide 3 Note Mid Only").ToPtr();
  m_centerAmpSideMidOnlyAuto =
      pathplanner::PathPlannerAuto("Center-AmpSide 3 Note Mid Only").ToPtr();

  m_getOutSourceSide =
      pathplanner::PathPlannerAuto("Get Out SourceSide").ToPtr();

  m_SourceSideMidOnlyLandCenterR =
      pathplanner::PathPlannerAuto("SourceSide 3 Note Mid Only Center Last")
          .ToPtr();

  m_straightLine = pathplanner::PathPlannerAuto("straight line test").ToPtr();
  m_squarePath = pathplanner::PathPlannerAuto("sqare test").ToPtr();
  m_nonoPath = pathplanner::PathPlannerAuto("rest in peace robot").ToPtr();

  /**
   * Automatic pathfinding triggers. Still need to test.
   * IF I SEE THESE ENABLED DURING A MATCH I WILL BAN YOU FROM THE GITHUB
   * ORGANISATION >:(
   *                  -- Visvam.
   */
  // SourcePathTrigger.WhileTrue(m_SourcePath.get());

  // AmpPathTrigger.WhileTrue(m_AmpShotPath.get());

  // SubPathTrigger.WhileTrue(m_CenterSubPath.get());

  // Add loaded autons to the configurator.

  m_chooser.SetDefaultOption("Default Auto: Shoot Preload",
                             m_defaultAuto.get());

  m_chooser.AddOption("AmpSide Subwoofer 3 Note Auto",
                      m_AmpSide3NoteAuto.get());
  m_chooser.AddOption("SourceSide Subwoofer 3 Note Auto",
                      m_SourceSide3NoteAuto.get());
  m_chooser.AddOption("Center Subwoofer 3 Note Auto", m_center3NoteAuto.get());
  m_chooser.AddOption("Center Subwoofer 2 Note Auto", m_center2NoteAuto.get());

  m_chooser.AddOption("SourceSide Subwoofer 2 Note Auto",
                      m_SourceSide2NoteAuto.get());

  m_chooser.AddOption("AmpSide Subwoofer 2 Note Auto",
                      m_AmpSide2NoteAuto.get());

  m_chooser.AddOption("CenterSourceSide Mid Only 3 Note Auto",
                      m_centerSourceSideMidOnlyAuto.get());

  m_chooser.AddOption("CenterAmpSide Mid Only 3 Note Auto",
                      m_centerAmpSideMidOnlyAuto.get());

  m_chooser.AddOption("SourceSide Mid Only 3 Note Auto",
                      m_SourceSideMidOnlyAuto.get());

  m_chooser.AddOption("SourceSide Mid Only Inner First 3 Note Auto",
                      m_SourceSideMidInnerOnlyAuto.get());

  m_chooser.AddOption("AmpSide Mid Only 3 Note Auto",
                      m_AmpSideMidOnlyAuto.get());

  m_chooser.AddOption("Source Path", m_SourcePath.get());
  m_chooser.AddOption("Amp Path", m_AmpShotPath.get());
  m_chooser.AddOption("Sub Path", m_CenterSubPath.get());

  m_chooser.AddOption("SourceSide 3 Note Mid Only Center Last",
                      m_SourceSideMidOnlyLandCenterR.get());

  m_chooser.AddOption("square path test", m_squarePath.get());

  m_chooser.AddOption("5m straight line test", m_straightLine.get());

  // m_chooser.AddOption(
  //     "DO NOT RUN: the` forbidden auton (robot will explode if you run)",
  //     m_nonoPath.get());

  frc::SmartDashboard::PutData(&m_chooser);
}

void RobotContainer::ConfigureDashboard() {
  m_intake.InitVisualization(&m_mech_sideview);
  m_shooter.InitVisualization(&m_mech_sideview);

  frc::SmartDashboard::PutData("Mechanisms", &m_mech_sideview);
  frc::SmartDashboard::PutData("Intake", &m_intake);
  frc::SmartDashboard::PutData("Shooter", &m_shooter);
  frc::SmartDashboard::PutData("Drivebase", &m_swerve);
  frc::SmartDashboard::PutData(&m_chooser);
}

void RobotContainer::ConfigureAuto() {
  pathplanner::PathPlannerLogging::setLogActivePathCallback(
      [this](auto &&activePath) {
        m_swerve.GetField().GetObject("Hopper")->SetPoses(activePath);
      });
}

std::optional<frc::Rotation2d> RobotContainer::GetRotationTargetOverride() {
  if (m_intake.IsIntaking())
    return m_isRed ? 180_deg : 0_deg;
  else
    return std::nullopt;
}

frc2::Command *RobotContainer::GetAutonomousCommand() {
  return m_chooser.GetSelected();
}

frc2::CommandPtr RobotContainer::GetDisabledCommand() {
  return frc2::cmd::None();
}