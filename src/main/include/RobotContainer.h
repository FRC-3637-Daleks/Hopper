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
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/JoystickButton.h>

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

constexpr auto kMaxSpeed = 4.5_mps;
constexpr auto kMaxAcceleration = 6_mps_sq;
constexpr auto kPathMaxAcceleration = 4_mps_sq;
// Swerve Constants (NEED TO BE INTEGRATED)
// constexpr auto kMaxSpeed = ModuleConstants::kPhysicalMaxSpeed / 3; // left
// out as these are repeat values constexpr auto kMaxAcceleration = 10_fps_sq;
constexpr auto kMaxAngularSpeed = std::numbers::pi * 1_rad_per_s;
constexpr auto kMaxAngularAcceleration = std::numbers::pi * 2_rad_per_s_sq;

// XXX Very untrustworthy placeholder values.
constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

// Trapezoidal motion profile for the robot heading.
const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints{kMaxAngularSpeed, kMaxAngularAcceleration};
} // namespace AutoConstants

namespace OperatorConstants {

constexpr int kCopilotControllerPort = 1;
constexpr int kSwerveControllerPort = 0;

constexpr double kDeadband = 0.08;
constexpr double kClimbDeadband = 0.08;

constexpr int kStrafeAxis = frc::XboxController::Axis::kLeftX;
constexpr int kForwardAxis = frc::XboxController::Axis::kLeftY;
constexpr int kRotationAxis = frc::XboxController::Axis::kRightX;
constexpr int kFieldRelativeButton = frc::XboxController::Button::kRightBumper;

constexpr int kIntakeSourcePOV = 45;
constexpr int kIntakeGroundPOV = 90;
constexpr int kIntakeAMPPOV = 0;
constexpr int kIntakeShooterPOV = 270;
constexpr int kAutoIntake = 180;

constexpr frc::Pose2d kBlueSpeakerPose{0.112_m, 5.493_m, 0_deg};
constexpr frc::Pose2d kBlueAMPPose{1.812_m, 8.221_m, 0_deg};
constexpr frc::Pose2d kBlueStagePose{4.869_m, 4.144_m, 0_deg};
constexpr frc::Pose2d kBlueSourcePose{15.743_m, 0.410_m, 0_deg};
constexpr frc::Pose2d kRedSpeakerPose{16.362_m, 5.486_m, 0_deg};
constexpr frc::Pose2d kRedAMPPose{14.699_m, 8.221_m, 0_deg};
constexpr frc::Pose2d kRedStagePose{11.681_m, 4.144_m, 0_deg};
constexpr frc::Pose2d kRedSourcePose{0.676_m, 0.410_m, 0_deg};

constexpr frc::Pose2d kMidFarRNote{8.3_m, .77_m, 0_deg};
constexpr frc::Pose2d kMidRNote{8.3_m, 2.44_m, 0_deg};

constexpr frc::Pose2d kMidCNote{8.3_m, 4.1_m, 0_deg};

constexpr frc::Pose2d kMidLNote{8.3_m, 5.78_m, 0_deg};
constexpr frc::Pose2d kMidFarLNote{8.3_m, 7.43_m, 0_deg};

constexpr frc::Pose2d kBlueAmpShot{1.83_m, 7.55_m, 90_deg};
constexpr frc::Pose2d kRedAmpShot{14.7_m, 7.55_m, 90_deg};

constexpr frc::Pose2d kBlueSourcePickUp{15.3_m, 1.1_m, -60_deg};
constexpr frc::Pose2d kRedSourcePickUp{1.3_m, 1.1_m, -120_deg};

constexpr frc::Pose2d kBlueCenterSub{1.55_m, 5.55_m, 0_deg};
constexpr frc::Pose2d kRedCenterSub{15_m, 5.55_m, 180_deg};

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

  frc2::CommandJoystick m_swerveController{
      OperatorConstants::kSwerveControllerPort};

  // Button Triggers are defined here.

  frc2::Trigger m_slowModeTrigger{
      [this]() -> bool { return m_swerveController.GetRawButtonPressed(1); }};

  frc2::Trigger m_autoAmpTrigger{
      [this]() -> bool { return m_swerveController.GetRawButtonPressed(2); }};

  frc2::Trigger m_passMode{[this]() -> bool {
    return m_copilotController.GetLeftTriggerAxis() > 0.2;
  }};

  frc2::Trigger IdleIntakeTrigger{
      [this]() -> bool { return m_copilotController.GetPOV() == -1; }};
  /** Ground Intake Trigger, right on the Co-Pilot dpad*/
  frc2::Trigger GroundIntakeTrigger{[this]() -> bool {
    return m_copilotController.GetPOV() == OperatorConstants::kIntakeGroundPOV;
  }};
  /** Amp Intake Trigger, up on the Co-Pilot dpad*/
  frc2::Trigger AMPIntakeTrigger{[this]() -> bool {
    return m_copilotController.GetPOV() == OperatorConstants::kIntakeAMPPOV;
  }};
  /** Speaker Intake Trigger, left on the Co-Pilot dpad*/
  frc2::Trigger SpeakerIntakeTrigger{[this]() -> bool {
    return m_copilotController.GetPOV() == OperatorConstants::kIntakeShooterPOV;
  }};
  /** Auto Intake Trigger, down on the Co-Pilot dpad*/
  frc2::Trigger AutoIntakeTrigger{[this]() -> bool {
    return m_copilotController.GetPOV() == OperatorConstants::kAutoIntake;
  }};
  /** Source Intake Trigger, up & right on the Co-Pilot dpad*/
  frc2::Trigger SourceIntakeTrigger{[this]() -> bool {
    return m_copilotController.GetPOV() == OperatorConstants::kIntakeSourcePOV;
  }};

  /**Pit Reset Trigger, Start Button on Co-Pilot controller*/
  frc2::Trigger PitReset{
      [this]() -> bool { return m_copilotController.GetStartButton(); }};

  /**Flywheel off Trigger, back Button on Co-Pilot controller*/
  // DEPRICATED
  frc2::Trigger flywheelOffTrigger{
      [this]() -> bool { return m_copilotController.GetBackButton(); }};

  /** Source Path Trigger, right on the Pilot dpad*/
  frc2::Trigger SourcePathTrigger{
      [this]() -> bool { return m_swerveController.GetPOV() == 90; }};
  /** Amp Path Trigger, left on the Pilot dpad*/
  frc2::Trigger AmpPathTrigger{
      [this]() -> bool { return m_swerveController.GetPOV() == 270; }};
  /** Sub Path Trigger, up on the Pilot dpad*/
  frc2::Trigger SubPathTrigger{
      [this]() -> bool { return m_swerveController.GetPOV() == 0; }};

  frc2::Trigger DriveFwdTrigger{
      [this]() -> bool { return m_swerveController.GetPOV() == 0; }};

  frc2::Trigger DriveStrafeLeftTrigger{
      [this]() -> bool { return m_swerveController.GetPOV() == 270; }};

  frc2::Trigger DriveStrafeRightTrigger{
      [this]() -> bool { return m_swerveController.GetPOV() == 90; }};

  frc2::Trigger DriveRevTrigger{
      [this]() -> bool { return m_swerveController.GetPOV() == 180; }};

  // The robot's subsystems are defined here...

  Shooter m_shooter;
  Drivetrain m_swerve;
  Intake m_intake;
  Climb m_climb;
  Vision m_vision;

  // The autonomous commands are initialized here.

  frc2::CommandPtr m_defaultAuto{frc2::cmd::None()};

  frc2::CommandPtr m_AmpSide3NoteAuto{frc2::cmd::None()};
  frc2::CommandPtr m_center3NoteAuto{frc2::cmd::None()};
  frc2::CommandPtr m_SourceSide3NoteAuto{frc2::cmd::None()};
  frc2::CommandPtr m_AmpSide2NoteAuto{frc2::cmd::None()};
  frc2::CommandPtr m_center2NoteAuto{frc2::cmd::None()};
  frc2::CommandPtr m_SourceSide2NoteAuto{frc2::cmd::None()};
  frc2::CommandPtr m_AmpSideMidOnlyAuto{frc2::cmd::None()};
  frc2::CommandPtr m_SourceSideMidOnlyAuto{frc2::cmd::None()};
  frc2::CommandPtr m_SourceSideMidInnerOnlyAuto{frc2::cmd::None()};
  frc2::CommandPtr m_centerAmpSideMidOnlyAuto{frc2::cmd::None()};
  frc2::CommandPtr m_centerSourceSideMidOnlyAuto{frc2::cmd::None()};

  frc2::CommandPtr m_SourceSideMidOnlyInnerFirst{frc2::cmd::None()};
  frc2::CommandPtr m_SourceSideMidOnlyCenterFirst{frc2::cmd::None()};

  frc2::CommandPtr m_getOutSourceSide{frc2::cmd::None()};
  frc2::CommandPtr m_SourcePath{frc2::cmd::None()};
  frc2::CommandPtr m_AmpShotPath{frc2::cmd::None()};
  frc2::CommandPtr m_CenterSubPath{frc2::cmd::None()};

  // Odometry Testing Paths
  frc2::CommandPtr m_straightLine{frc2::cmd::None()};
  frc2::CommandPtr m_squarePath{frc2::cmd::None()};
  frc2::CommandPtr m_nonoPath{frc2::cmd::None()};

  frc2::CommandPtr m_ampLineUp{frc2::cmd::None()};

  frc::SendableChooser<frc2::Command *> m_chooser;
  /** Checks if FRC driverstation is configured for red or blue side*/
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