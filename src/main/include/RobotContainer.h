// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/MathUtil.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <numbers>

#include "subsystems/Shooter.h"
#include "subsystems/Drivetrain.h"


namespace AutoConstants {

constexpr auto kMaxSpeed = 1_mps;
constexpr auto kMaxAcceleration = units::feet_per_second_squared_t{10};


// Swerve Constants (NEED TO BE INTEGRATED)
// constexpr auto kMaxSpeed = ModuleConstants::kPhysicalMaxSpeed / 3; // left
// out as these are repeat values constexpr auto kMaxAcceleration = 10_fps_sq;
constexpr auto kMaxAngularSpeed = 180_rpm;
constexpr auto kMaxAngularAcceleration = std::numbers::pi * 1_rad_per_s_sq;

// XXX Very untrustworthy placeholder values.
constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

// Trapezoidal motion profile for the robot heading.
const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints{kMaxAngularSpeed, kMaxAngularAcceleration};

} // namespace AutoConstants


namespace OperatorConstants {

constexpr int kDriverControllerPort = 0;
constexpr int kSwerveControllerPort = 0;
constexpr int kInputControllerPort = 1;

constexpr double kDeadband = 0.08;

constexpr int kStrafeAxis = frc::XboxController::Axis::kLeftX;
constexpr int kForwardAxis = frc::XboxController::Axis::kLeftY;
constexpr int kRotationAxis = frc::XboxController::Axis::kRightX;
constexpr int kFieldRelativeButton = frc::XboxController::Button::kRightBumper;

}  // namespace OperatorConstants

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

  frc2::CommandPtr GetAutonomousCommand();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_driverController{
      OperatorConstants::kDriverControllerPort};

  frc2::CommandXboxController m_swerveController{
      OperatorConstants::kSwerveControllerPort};

  frc2::CommandXboxController m_intakeController{
      OperatorConstants::kInputControllerPort};

  // The robot's subsystems are defined here...
  
  //Shooter m_subsystem;
  Drivetrain m_swerve;

  void ConfigureBindings();
};