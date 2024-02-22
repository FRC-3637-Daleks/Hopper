// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>

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

  constexpr auto target = [] () -> frc::Pose2d { return {-2_m, 0_m, 0_rad}; }; //implement live apriltag targeting

  m_swerve.SetDefaultCommand(m_swerve.SwerveCommandFieldRelative(fwd, strafe, rot));

  m_swerveController.A()
      .OnTrue(m_swerve.ZeroHeadingCommand());

  m_swerveController.B()
      .WhileTrue(m_swerve.TurnToAngleCommand(45_deg));

  m_swerveController.X()
    .WhileTrue(m_swerve.ZTargetPoseCommand(target, fwd, strafe));

  m_swerveController.Y()
      .WhileTrue(m_swerve.SwerveSlowCommand(fwd,strafe,rot));

  // m_swerveController.LeftBumper()
  //     .WhileTrue(m_swerve.ConfigAbsEncoderCommand());
  
  m_swerveController.RightBumper()
      .WhileTrue(m_swerve.SwerveCommand(fwd, strafe, rot));
  
  //m_swerveController.X().WhileTrue(m_swerve.ZeroAbsEncodersCommand());
  // m_swerveController.LeftBumper().WhileTrue(m_swerve.ConfigAbsEncoderCommand());
      
  //Configure Shooter Bindings.
  auto flywheel = [this] () -> double {
    return m_copilotController.GetRightTriggerAxis();
  };

  auto pivot = [this] () -> units::degree_t {
    return (ShooterConstants::kMaxAngle - ShooterConstants::kMinAngle) * 
            frc::ApplyDeadband(m_copilotController.GetLeftY(), OperatorConstants::kDeadband) + ShooterConstants::kMinAngle;
  };

  m_shooter.SetDefaultCommand(m_shooter.ShooterCommand(flywheel, pivot));

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
      std::pair<int, frc2::CommandPtr>{OperatorConstants::kIntakeShooterPOV, m_intake.IntakeArmSpeakerCommand(false)}
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