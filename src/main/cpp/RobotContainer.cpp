// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>
#include <frc/geometry/Pose3d.h>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  frc::DataLogManager::LogNetworkTables(true);

  // Configure the button bindings
  ConfigureBindings();

  // Configure Dashboard
  ConfigureDashboard();
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

  auto targetSpeaker = [this] () -> frc::Pose2d { return m_isRed ? OperatorConstants::kRedSpeakerPose : OperatorConstants::kBlueSpeakerPose; }; //implement live apriltag targeting
  auto targetAMP = [this] () -> frc::Pose2d { return m_isRed ? OperatorConstants::kRedAMPPose : OperatorConstants::kBlueAMPPose; }; //implement live apriltag targeting
  auto targetStage = [this] () -> frc::Pose2d { return m_isRed ? OperatorConstants::kRedStagePose : OperatorConstants::kBlueStagePose; }; //implement live apriltag targeting
  auto targetSource = [this] () -> frc::Pose2d { return m_isRed ? OperatorConstants::kRedSourcePose : OperatorConstants::kBlueSourcePose; }; //implement live apriltag targeting

  constexpr auto SubWoofer = [] () -> frc::Pose2d { return {-2_m, 0_m, 0_rad}; };

  m_swerve.SetDefaultCommand(m_swerve.SwerveCommandFieldRelative(fwd, strafe, rot));

  m_swerveController.Start()
      .OnTrue(m_swerve.ZeroHeadingCommand());

  m_swerveController.A()
    .WhileTrue(m_swerve.ZTargetPoseCommand(targetSource, fwd, strafe));

  m_swerveController.B()
    .WhileTrue(m_swerve.ZTargetPoseCommand(targetSpeaker, fwd, strafe));

  m_swerveController.X()
    .WhileTrue(m_swerve.ZTargetPoseCommand(targetAMP, fwd, strafe));

  m_swerveController.Y()
    .WhileTrue(m_swerve.ZTargetPoseCommand(targetStage, fwd, strafe));

  m_swerveController.LeftStick()
      .WhileTrue(m_swerve.SwerveSlowCommand(fwd,strafe,rot));
  
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
    return m_copilotController.GetRightTriggerAxis();
  };

  auto pivot = [this] () -> units::degrees_per_second_t {
    return 16_deg_per_s * frc::ApplyDeadband(m_copilotController.GetLeftY(), OperatorConstants::kDeadband);
  };

<<<<<<< HEAD
  m_shooter.SetDefaultCommand(
      m_shooter.ShooterCommand(
          static_cast<std::function<double()>>(flywheel),
          static_cast<std::function<units::angular_velocity::degrees_per_second_t()>>(pivot)
      )
  );
  
=======
  auto calculateDistance = [this]() -> units::meter_t {
      frc::Pose2d RobotPose2d = m_swerve.GetPose();
      
      // Determine the IDs of the speaker AprilTags based on the alliance color
      int id = m_isRed ? 4 : 7;

      // Get the pose of the speaker AprilTag based on its ID
      frc::Pose3d SpeakerPose = m_aprilTagFieldLayout.GetTagPose(id).value();
      SpeakerPose = frc::Pose3d{SpeakerPose.X(), SpeakerPose.Y(), SpeakerPose.Z() + 0.5_m, SpeakerPose.Rotation()};
      // auto it1 = m_aprilTagFieldLayout.GetTagPose(id);
      // if (it1.has_value()) {
      //     SpeakerPose = it1.value();
      // } else {
      //     // Handle case where neither tag is found
      //     SpeakerPose = frc::Pose3d();
      // }
      
      units::meter_t z = 1.5_ft; // estimation of shooter height
      
      // Construct Pose3d using the constructor that takes a Pose2d
      frc::Pose3d RobotPose3d{RobotPose2d.Translation().X(), RobotPose2d.Translation().Y(), z, frc::Rotation3d{0_deg, RobotPose2d.Rotation().Degrees(), 0_deg}};
      // frc::SmartDashboard::PutData(RobotPose3d);
      // Calculate the horizontal distance between RobotPose and SpeakerPose
      units::meter_t offset = RobotPose3d.Translation().Distance(SpeakerPose.Translation());
      return offset; //Return the horizontal distance as units::meter_t
  };

  m_shooter.SetDefaultCommand(m_shooter.ShooterCommand(flywheel, calculateDistance));

>>>>>>> origin/visvam-wip
  // Configure Intake Bindings.
  auto position = [this]() -> int {
    return m_copilotController.GetPOV();
  };

  m_intake.SetDefaultCommand(
    frc2::cmd::Select<int>(
      position,
      std::pair<int, frc2::CommandPtr>{-1, m_intake.IdleIntakeCommand()},
      std::pair<int, frc2::CommandPtr>{OperatorConstants::kIntakeGroundPOV, m_intake.IntakeArmIntakeCommand(false)},
      std::pair<int, frc2::CommandPtr>{OperatorConstants::kIntakeAMPPOV, m_intake.IntakeArmAMPCommand(false)},
      std::pair<int, frc2::CommandPtr>{OperatorConstants::kIntakeShooterPOV, m_intake.IntakeArmSpeakerCommand(false)},
      std::pair<int, frc2::CommandPtr>{OperatorConstants::kAutoIntake, m_intake.IntakeRing()}
      )
  );

  m_copilotController.A()
    .WhileTrue(m_intake.IntakeIn());
    
  m_copilotController.B()
    .WhileTrue(m_intake.IntakeOut());

  auto climb = [this] () -> double { return -frc::ApplyDeadband(m_copilotController.GetRightY(), OperatorConstants::kDeadband); };

  m_climb.SetDefaultCommand(m_climb.ClimbCommand(climb)); 
}

void RobotContainer::ConfigureDashboard()
{
  m_intake.InitVisualization(&m_mech_sideview);
  m_shooter.InitVisualization(&m_mech_sideview);

  frc::SmartDashboard::PutData("Mechanisms", &m_mech_sideview);
}


frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  // You can ignore this for now.
  //return autos::ExampleAuto(&m_subsystem);
  return frc2::cmd::Idle();
}

frc2::CommandPtr RobotContainer::GetDisabledCommand(){
  // return m_swerve.CoastModeCommand(true).IgnoringDisable(true);
  return frc2::cmd::None();
}

