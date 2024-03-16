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

  constexpr auto targetMidFarRNote = []() -> frc::Pose2d {
    return OperatorConstants::kMidFarRNote;
  }; // implement live apriltag targeting

  constexpr auto targetMidRNote = []() -> frc::Pose2d {
    return OperatorConstants::kMidRNote;
  }; // implement live apriltag targeting

  constexpr auto targetMidCNote = []() -> frc::Pose2d {
    return OperatorConstants::kMidCNote;
  }; // implement live apriltag targeting

  constexpr auto targetMidLNote = []() -> frc::Pose2d {
    return OperatorConstants::kMidLNote;
  }; // implement live apriltag targeting

  constexpr auto targetMidFarLNote = []() -> frc::Pose2d {
    return OperatorConstants::kMidFarLNote;
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

  AMPIntakeTrigger.OnTrue(m_intake.IntakeFromPlayerStation());

  SpeakerIntakeTrigger.OnTrue(m_intake.IntakeArmSpeakerCommand(true));

  AutoIntakeTrigger.OnTrue(m_intake.IntakeRing());

  // SourceIntakeTrigger.OnTrue(m_intake.IntakeFromPlayerStation());

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
      pathplanner::ReplanningConfig(true, true, 1_m, .25_m);

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

  pathplanner::NamedCommands::registerCommand(
      "ShootAmp", m_intake.ShootOnAMP().WithName("ShootAMP"));

  pathplanner::NamedCommands::registerCommand(
      "IntakeRing", m_intake.IntakeRing().WithName("IntakeRing"));

  pathplanner::NamedCommands::registerCommand(
      "OutputToShooterZTarget",
      frc2::cmd::Sequence(
          m_swerve
              .ZTargetPoseCommand(targetSpeaker, fwd, strafe, true, alliance)
              .WithTimeout(.5_s),
          m_intake.OutputToShooter().WithName("OutputToShooterZTarget")));

  pathplanner::NamedCommands::registerCommand(
      "OutputToShooter",
      m_intake.OutputToShooter().WithName("OutputToShooter"));

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
  m_centerSourceSideMidOnlyAuto =
      pathplanner::PathPlannerAuto("Center-SourceSide 3 Note Mid Only").ToPtr();
  m_centerAmpSideMidOnlyAuto =
      pathplanner::PathPlannerAuto("Center-AmpSide 3 Note Mid Only").ToPtr();

  m_getOutSourceSide =
      pathplanner::PathPlannerAuto("Get Out SourceSide").ToPtr();

  // SourcePathTrigger.WhileTrue(m_SourcePath.get());

  // AmpPathTrigger.WhileTrue(m_AmpShotPath.get());   Not reliable enough to
  // risk this

  // SubPathTrigger.WhileTrue(m_CenterSubPath.get());

  m_chooser.SetDefaultOption("AmpSide Subwoofer 3 Note Auto",
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

  m_chooser.AddOption("AmpSide Mid Only 3 Note Auto",
                      m_AmpSideMidOnlyAuto.get());

  m_chooser.AddOption("Source Path", m_SourcePath.get());
  m_chooser.AddOption("Amp Path", m_AmpShotPath.get());
  m_chooser.AddOption("Sub Path", m_CenterSubPath.get());

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