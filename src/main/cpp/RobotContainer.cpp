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
    return DriveConstants::kMaxTurnRate * squaredInput;
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

  m_swerveController.Button(12).OnTrue(m_swerve.ZeroHeadingCommand());

  m_swerveController.Button(7).WhileTrue(
      m_swerve.ZTargetPoseCommand(targetSource, fwd, strafe, false, checkRed));

  m_swerveController.Button(4).WhileTrue(
      m_swerve.ZTargetPoseCommand(targetSpeaker, fwd, strafe, true, checkRed));

  m_swerveController.Button(3).WhileTrue(
      m_swerve.ZTargetPoseCommand(targetAMP, fwd, strafe, true, checkRed));

  m_swerveController.Button(8).WhileTrue(
      m_swerve.ZTargetPoseCommand(targetStage, fwd, strafe, false, checkRed));

  m_slowModeTrigger.WhileTrue(
      m_swerve.SwerveSlowCommand(fwd, strafe, rot, checkRed));

  m_swerveController.Button(9).ToggleOnTrue(
      m_swerve.SwerveCommand(fwd, strafe, rot));

  constexpr auto one_meter = []() -> units::meters_per_second_t {
    return 1_mps;
  };

  constexpr auto neg_one_meter = []() -> units::meters_per_second_t {
    return -1_mps;
  };

  // Precise driving commands.

  // Configure Shooter Bindings.

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

  // Configure Intake Bindings.

  // Manual intake using percent out.

  // Manual Intake In/Out.

  constexpr auto flywheelOff = []() { return 0.0; };

  // Configure climb bindings.

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
          pathplanner::PIDConstants(7.0, 0.0, 0.0), // Translation constants
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
        this->m_swerve.Drive(speed.vx, speed.vy, speed.omega, false, false);
      },
      pathFollowerConfig,
      [this]() { return m_isRed; }, // replace later, just a placeholder
      (&m_swerve));

  pathplanner::PathConstraints constraints = pathplanner::PathConstraints(
      AutoConstants::kMaxSpeed, AutoConstants::kPathMaxAcceleration,
      AutoConstants::kMaxAngularSpeed, AutoConstants::kMaxAngularAcceleration);

  // Register named commands for use in auton.

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

  m_SourceSideMidOnlyInnerFirst =
      pathplanner::PathPlannerAuto("SourceSide 3 Note Mid Only Inner First")
          .ToPtr();

  m_SourceSideMidOnlyCenterFirst =
      pathplanner::PathPlannerAuto("SourceSide 3 Note Mid Only Center First")
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
  SourcePathTrigger.WhileTrue(m_SourcePath.get());

  AmpPathTrigger.WhileTrue(m_AmpShotPath.get());

  SubPathTrigger.WhileTrue(m_CenterSubPath.get());

  //   Add loaded autons to the configurator.

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
                      m_SourceSideMidOnlyInnerFirst.get());
  m_chooser.AddOption("SourceSide 3 Note Mid Only Center First",
                      m_SourceSideMidOnlyCenterFirst.get());

  m_chooser.AddOption("square path test", m_squarePath.get());

  m_chooser.AddOption("5m straight line test", m_straightLine.get());

  // m_chooser.AddOption(
  //     "DO NOT RUN: the` forbidden auton (robot will explode if you run)",
  //     m_nonoPath.get());

  frc::SmartDashboard::PutData(&m_chooser);
}

void RobotContainer::ConfigureDashboard() {
  frc::SmartDashboard::PutData("Drivebase", &m_swerve);
  frc::SmartDashboard::PutData(&m_chooser);
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