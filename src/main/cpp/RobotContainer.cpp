// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <units/math.h>

#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>

#include "commands/Autos.h"
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

  //


 const pathplanner::HolonomicPathFollowerConfig pathFollowerConfig = pathplanner::HolonomicPathFollowerConfig(
    pathplanner::PIDConstants(1.0, 0.0, 0.0), // Translation constants 
    pathplanner::PIDConstants(1.0, 0.0, 0.0), // Rotation constants 
    AutoConstants::kMaxSpeed,
    DriveConstants::kRadius, // Drive base radius (distance from center to furthest module) 
    pathplanner::ReplanningConfig()
);






RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {

    // HopperAuto = pathplanner::PathPlannerAuto("Hopper").ToPtr().Unwrap();
    // frc::SmartDashboard::PutData("Hopper", HopperAuto.get());
  //m_subsystem.SetDefaultCommand(m_subsystem.FlywheelCommand(m_driverController.GetLeftY()));

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

  auto target = [this] () -> frc::Pose2d { return {-2_m, 0_m, 0_rad}; }; //implement live apriltag targeting

  m_swerve.SetDefaultCommand(m_swerve.SwerveCommand(fwd, strafe, rot));
  m_swerveController.Button(OperatorConstants::kFieldRelativeButton)
      .WhileTrue(m_swerve.SwerveCommandFieldRelative(fwd, strafe, rot));


      pathplanner::AutoBuilder::configureHolonomic(
        [this](){return this->m_swerve.GetPose();},
        [this](frc::Pose2d pose){this->m_swerve.ResetOdometry(pose);},
        [this](){return this->m_swerve.GetSpeed();},
        [this](frc::ChassisSpeeds speed){this->m_swerve.Drive(
        speed.vx, speed.vy, speed.omega, false);},
        pathFollowerConfig,
        [](){  

          auto alliance = frc::DriverStation::GetAlliance();
          if(alliance) {
            return alliance.value() == frc::DriverStation::Alliance::kRed;
          }

          return false;


          
        }, //replace later, just a placeholder
        (&m_swerve)
      );
  // m_swerve.SetDefaultCommand(m_swerve.SwerveCommand(fwd, strafe, rot));
  //  m_swerveController.Button(OperatorConstants::kFieldRelativeButton)
  //     .WhileTrue(m_swerve.SwerveCommandFieldRelative(fwd, strafe, rot));


      pathplanner::PathPoint MidFarL({7.0_m, 7.40_m});
      pathplanner::PathPoint MidL({7.0_m, 5.75_m});
      pathplanner::PathPoint MidCenter({7.0_m, 4.1_m});
      pathplanner::PathPoint MidR({7.0_m, 2.45_m});
      pathplanner::PathPoint MidFarR({7.0_m, 0.75_m});

      pathplanner::PathPoint CloseL({1.6_m, 5.55_m});
      pathplanner::PathPoint CloseCenter({1.6_m, 7.0_m});
      pathplanner::PathPoint CloseR({1.6_m, 4.1_m});

      pathplanner::NamedCommands::registerCommand("RevShooter", frc2::cmd::Print("Revving Shooter"));
      pathplanner::NamedCommands::registerCommand("ShootNote", frc2::cmd::Print("Shooting Note"));
      pathplanner::NamedCommands::registerCommand("RevIntake", frc2::cmd::Print("Revving Intake"));
      pathplanner::NamedCommands::registerCommand("StopIntake", frc2::cmd::Print("Stopping Intake"));
      
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
      std::pair<int, frc2::CommandPtr>{OperatorConstants::kIntakeGroundPOV, m_intake.IntakeArmIntakeCommand()},
      std::pair<int, frc2::CommandPtr>{OperatorConstants::kIntakeAMPPOV, m_intake.IntakeArmAMPCommand()},
      std::pair<int, frc2::CommandPtr>{OperatorConstants::kIntakeShooterPOV, m_intake.IntakeArmSpeakerCommand()}
    )
  );

  m_copilotController.A()
    .WhileTrue(m_intake.IntakeIn());
    
  m_copilotController.B()
    .WhileTrue(m_intake.IntakeOut());

  auto climb = [this] () -> double { return frc::ApplyDeadband(m_copilotController.GetRightY(), OperatorConstants::kDeadband); };

  m_climb.SetDefaultCommand(m_climb.ClimbCommand(climb));

}


frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  // You can ignore this for now.
  //return autos::ExampleAuto(&m_subsystem);
  return pathplanner::PathPlannerAuto("Hopper").ToPtr();
}
