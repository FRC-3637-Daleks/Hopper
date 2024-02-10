// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>

#include "commands/Autos.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {

// Configure Swerve Bindings.
  auto fwd = [this]() -> units::meters_per_second_t {
    return (DriveConstants::kMaxTeleopSpeed *
            frc::ApplyDeadband(
                -m_swerveController.GetRawAxis(OperatorConstants::kForwardAxis),
                OperatorConstants::kDeadband));
  };

  auto strafe = [this]() -> units::meters_per_second_t {
    return (DriveConstants::kMaxTeleopSpeed *
            frc::ApplyDeadband(
                -m_swerveController.GetRawAxis(OperatorConstants::kStrafeAxis),
                OperatorConstants::kDeadband));
  };

  auto rot = [this]() -> units::revolutions_per_minute_t {
    return (AutoConstants::kMaxAngularSpeed *
            frc::ApplyDeadband(
                -m_swerveController.GetRawAxis(OperatorConstants::kRotationAxis),
                OperatorConstants::kDeadband));
  };

  auto target = [this] () -> frc::Pose2d { return {-2_m, 0_m, 0_rad}; }; //implement live apriltag targeting

  m_swerve.SetDefaultCommand(m_swerve.SwerveCommandFieldRelative(fwd, strafe, rot));

  m_swerveController.RightBumper()
      .WhileTrue(m_swerve.SwerveCommand(fwd, strafe, rot));

  m_swerveController.A()
      .OnTrue(m_swerve.ZeroHeadingCommand());

  m_swerveController.B()
      .WhileTrue(m_swerve.TurnToAngleCommand(45_deg));

  m_swerveController.X()
    .WhileTrue(m_swerve.ZTargetPoseCommand(target, fwd, strafe));

  m_swerveController.LeftBumper()
      .WhileTrue(m_swerve.ConfigAbsEncoderCommand());

  //Configure Shooter Bindings.
  auto flywheel = [this] () -> double {
    return frc::ApplyDeadband(m_driverController.GetLeftY(), OperatorConstants::kDeadband);
  };

  auto pivot = [this] () -> units::degree_t {
    return (ShooterConstants::kMaxAngle - ShooterConstants::kMinAngle) * 
            m_driverController.GetRightTriggerAxis() + ShooterConstants::kMinAngle;
  };

  m_shooter.SetDefaultCommand(m_shooter.ShooterCommand(flywheel, pivot));
}


frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  // You can ignore this for now.
  //return autos::ExampleAuto(&m_subsystem);
  return frc2::cmd::Idle();
}
