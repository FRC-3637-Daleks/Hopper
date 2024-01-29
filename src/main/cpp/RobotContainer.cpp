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
  const auto kRadius = 17.5_in;

 const pathplanner::HolonomicPathFollowerConfig pathFollowerConfig = pathplanner::HolonomicPathFollowerConfig(
    pathplanner::PIDConstants(5.0, 0.0, 0.0), // Translation constants 
    pathplanner::PIDConstants(5.0, 0.0, 0.0), // Rotation constants 
    AutoConstants::kMaxSpeed,
    kRadius, // Drive base radius (distance from center to furthest module) 
    pathplanner::ReplanningConfig()
);



RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  //m_subsystem.SetDefaultCommand(m_subsystem.FlywheelCommand(m_driverController.GetLeftY()));

  auto fwd = [this]() -> units::meters_per_second_t {
    return (DriveConstants::kMaxTeleopSpeed *
            frc::ApplyDeadband(
                m_swerveController.GetRawAxis(OperatorConstants::kForwardAxis),
                OperatorConstants::kDeadband));
  };
  auto strafe = [this]() -> units::meters_per_second_t {
    return (DriveConstants::kMaxTeleopSpeed *
            frc::ApplyDeadband(
                m_swerveController.GetRawAxis(OperatorConstants::kStrafeAxis),
                OperatorConstants::kDeadband));
  };

  auto rot = [this]() -> units::revolutions_per_minute_t {
    return (AutoConstants::kMaxAngularSpeed *
            frc::ApplyDeadband(
                m_swerveController.GetRawAxis(OperatorConstants::kRotationAxis),
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


      
  m_swerve.SetDefaultCommand(m_swerve.SwerveCommandFieldRelative(fwd, strafe, rot));
  m_swerveController.Button(OperatorConstants::kFieldRelativeButton).WhileTrue(m_swerve.SwerveCommand(fwd, strafe, rot));

  m_swerveController.A()
      .OnTrue(m_swerve.ZeroHeadingCommand());

}


frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  // You can ignore this for now.
  //return autos::ExampleAuto(&m_subsystem);
  return pathplanner::PathPlannerAuto("Hopper").ToPtr();
}
