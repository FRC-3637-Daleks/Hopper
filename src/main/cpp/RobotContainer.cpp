// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>

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

      frc::Pose2d MidFarL(7.0_m, 7.40_m, 0_deg);
      frc::Pose2d MidL(7.0_m, 5.75_m, 0_deg);
      frc::Pose2d MidCenter(7.0_m, 4.1_m, 0_deg);
      frc::Pose2d MidR(7.0_m, 2.45_m, 0_deg);
      frc::Pose2d MidFarR(7.0_m, 0.75_m, 0_deg);

      frc::Pose2d CloseL(1.6_m, 5.55_m, 0_deg);
      frc::Pose2d CloseCenter(1.6_m, 7.0_m, 0_deg);
      frc::Pose2d CloseR(1.6_m, 4.1_m, 0_deg);

    
  m_swerve.SetDefaultCommand(m_swerve.SwerveCommandFieldRelative(fwd, strafe, rot));
  m_swerveController.RightBumper().WhileTrue(m_swerve.SwerveCommand(fwd, strafe, rot));
  // m_swerveController.Button(OperatorConstants::kFieldRelativeButton).WhileTrue(m_swerve.SwerveCommand(fwd, strafe, rot));

  m_swerveController.A()
      .OnTrue(m_swerve.ZeroHeadingCommand());

  //m_swerveController.X().WhileTrue(m_swerve.ZeroAbsEncodersCommand());
  m_swerveController.LeftBumper().WhileTrue(m_swerve.ConfigAbsEncoderCommand());

  m_swerveController.X().WhileTrue(m_swerve.SwerveCommand([] () -> units::meters_per_second_t { return 1_mps; }, [] () -> units::meters_per_second_t { return 0_mps; }, [] () -> units::radians_per_second_t { return 0_rad_per_s; }));
}


frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  // You can ignore this for now.
  //return autos::ExampleAuto(&m_subsystem);
  return pathplanner::PathPlannerAuto("Hopper").ToPtr();
}
