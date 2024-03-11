// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/MathUtil.h>
#include <frc/XboxController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "frc/apriltag/AprilTagFields.h"
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <numbers>

#include "subsystems/Climb.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"
#include "subsystems/Vision.h"

namespace AutoConstants {

constexpr auto kMaxSpeed = 4_mps;
constexpr auto kMaxAcceleration = 8_mps_sq;
// Swerve Constants (NEED TO BE INTEGRATED)
// constexpr auto kMaxSpeed = ModuleConstants::kPhysicalMaxSpeed / 3; // left
// out as these are repeat values constexpr auto kMaxAcceleration = 10_fps_sq;
constexpr auto kMaxAngularSpeed = 120_rpm;
constexpr auto kMaxAngularAcceleration = std::numbers::pi * 1_rad_per_s_sq;

// XXX Very untrustworthy placeholder values.
constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

// Trapezoidal motion profile for the robot heading.
const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints{kMaxAngularSpeed, kMaxAngularAcceleration};

constexpr pathplanner::PathConstraints DefaultConstraints(
    AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration,
    AutoConstants::kMaxAngularSpeed, AutoConstants::kMaxAngularAcceleration);

} // namespace AutoConstants

namespace OperatorConstants {

constexpr int kCopilotControllerPort = 1;
constexpr int kSwerveControllerPort = 0;

constexpr double kDeadband = 0.08;
constexpr double kClimbDeadband = 0.8;

constexpr int kStrafeAxis = frc::XboxController::Axis::kLeftX;
constexpr int kForwardAxis = frc::XboxController::Axis::kLeftY;
constexpr int kRotationAxis = frc::XboxController::Axis::kRightX;
constexpr int kFieldRelativeButton = frc::XboxController::Button::kRightBumper;

constexpr int kIntakeGroundPOV = 90;
constexpr int kIntakeAMPPOV = 0;
constexpr int kIntakeShooterPOV = 270;
constexpr int kAutoIntake = 180;

constexpr frc::Pose2d kBlueSpeakerPose{0.14_m, 5.5222_m, 0_deg};
constexpr frc::Pose2d kBlueAMPPose{1.812_m, 8.239_m, 0_deg};
constexpr frc::Pose2d kBlueStagePose{4.869_m, 4.144_m, 0_deg};
constexpr frc::Pose2d kBlueSourcePose{15.733_m, 0.410_m, 0_deg};
constexpr frc::Pose2d kRedSpeakerPose{16.336_m, 5.5222_m, 0_deg};
constexpr frc::Pose2d kRedAMPPose{14.622_m, 8.239_m, 0_deg};
constexpr frc::Pose2d kRedStagePose{11.681_m, 4.144_m, 0_deg};
constexpr frc::Pose2d kRedSourcePose{0.676_m, 0.410_m, 0_deg};

constexpr frc::Pose2d kCenterFarRNote{8.3_m, .77_m, 0_deg};
constexpr frc::Pose2d kCenterRNote{8.3_m, 2.44_m, 0_deg};

constexpr frc::Pose2d kCenterCNote{8.3_m, 4.1_m, 0_deg};

constexpr frc::Pose2d kCenterLNote{8.3_m, 5.78_m, 0_deg};
constexpr frc::Pose2d kCenterFarLNote{8.3_m, 7.43_m, 0_deg};

constexpr frc::Pose2d kBlueAmpShot{1.83_m, 7.6_m, 90_deg};
constexpr frc::Pose2d kRedAmpShot{14.7_m, 7.6_m, 90_deg};

constexpr frc::Pose2d kBlueSourcePickUp{15.4_m, 1_m, -60_deg};
constexpr frc::Pose2d kRedSourcePickUp{1.2_m, 1_m, -120_deg};

} // namespace OperatorConstants

namespace FieldConstants {

constexpr auto field_length = 54_ft + 3.25_in;
constexpr auto field_width = 26_ft + 11.75_in;
constexpr auto mid_line = field_length / 2;

constexpr frc::Pose2d feeder_station{{625_in, 12_in}, -80_deg};

constexpr auto near_note_separation = 57_in;
constexpr auto mid_note_separation = 66_in;
constexpr auto near_note_wall_dist = 114_in;
constexpr frc::Translation2d note_positions[] = {
    {near_note_wall_dist, field_width / 2},
    {near_note_wall_dist, field_width / 2 + near_note_separation},
    {near_note_wall_dist, field_width / 2 + 2 * near_note_separation},
    {field_length - near_note_wall_dist, field_width / 2},
    {field_length - near_note_wall_dist,
     field_width / 2 + near_note_separation},
    {field_length - near_note_wall_dist,
     field_width / 2 + 2 * near_note_separation},
    {mid_line, field_width / 2 + 2 * mid_note_separation},
    {mid_line, field_width / 2 + mid_note_separation},
    {mid_line, field_width / 2},
    {mid_line, field_width / 2 - mid_note_separation},
    {mid_line, field_width / 2 - 2 * mid_note_separation},
};

} // namespace FieldConstants

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
public:
  RobotContainer();
  frc2::CommandPtr GetDisabledCommand();
  frc2::Command *GetAutonomousCommand();
  std::unique_ptr<frc2::Command> HopperAuto;

public:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_copilotController{
      OperatorConstants::kCopilotControllerPort};

  frc2::CommandXboxController m_swerveController{
      OperatorConstants::kSwerveControllerPort};

  // The robot's subsystems are defined here...

  frc2::Trigger m_slowModeTrigger{[this]() -> bool {
    return m_swerveController.GetLeftTriggerAxis() > 0.2;
  }};

  frc2::Trigger m_manualIntake{[this]() -> bool {
    return m_copilotController.GetLeftTriggerAxis() > 0.2;
  }};

  frc2::Trigger IdleIntakeTrigger{
      [this]() -> bool { return m_copilotController.GetPOV() == -1; }};

  frc2::Trigger GroundIntakeTrigger{[this]() -> bool {
    return m_copilotController.GetPOV() == OperatorConstants::kIntakeGroundPOV;
  }};

  frc2::Trigger AMPIntakeTrigger{[this]() -> bool {
    return m_copilotController.GetPOV() == OperatorConstants::kIntakeAMPPOV;
  }};

  frc2::Trigger SpeakerIntakeTrigger{[this]() -> bool {
    return m_copilotController.GetPOV() == OperatorConstants::kIntakeShooterPOV;
  }};

  frc2::Trigger AutoIntakeTrigger{[this]() -> bool {
    return m_copilotController.GetPOV() == OperatorConstants::kAutoIntake;
  }};

  frc2::Trigger SourcePathTrigger{
      [this]() -> bool { return m_swerveController.GetPOV() == 90; }};
  frc2::Trigger AmpPathTrigger{
      [this]() -> bool { return m_swerveController.GetPOV() == 180; }};

  Shooter m_shooter;
  Drivetrain m_swerve;
  Intake m_intake;
  Climb m_climb;
  Vision m_vision;

  frc2::CommandPtr m_left3NoteAuto{frc2::cmd::None()};
  frc2::CommandPtr m_center3NoteAuto{frc2::cmd::None()};
  frc2::CommandPtr m_right3NoteAuto{frc2::cmd::None()};
  frc2::CommandPtr m_left2NoteAuto{frc2::cmd::None()};
  frc2::CommandPtr m_center2NoteAuto{frc2::cmd::None()};
  frc2::CommandPtr m_right2NoteAuto{frc2::cmd::None()};
  frc2::CommandPtr m_leftCenterOnlyAuto{frc2::cmd::None()};
  frc2::CommandPtr m_rightCenterOnlyAuto{frc2::cmd::None()};
  frc2::CommandPtr m_centerLeftCenterOnlyAuto{frc2::cmd::None()};
  frc2::CommandPtr m_centerRightCenterOnlyAuto{frc2::cmd::None()};
  frc2::CommandPtr m_getOutRight{frc2::cmd::None()};
  frc2::CommandPtr m_SourcePath{frc2::cmd::None()};
  frc2::CommandPtr m_AmpShotPath{frc2::cmd::None()};

  frc::SendableChooser<frc2::Command *> m_chooser;

  bool m_isRed;

  // AprilTag
  frc::AprilTagFieldLayout m_aprilTagFieldLayout =
      frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);

  // Global Dashboard Items
  frc::Mechanism2d m_mech_sideview{0.762, 0.660401016}; // scaled to meters

public:
  void ConfigureBindings();
  void ConfigureDashboard();
  void ConfigureAuto();
};
