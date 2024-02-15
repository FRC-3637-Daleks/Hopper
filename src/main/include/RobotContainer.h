// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/MathUtil.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/Mechanism2d.h>

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
#include "subsystems/Intake.h"
#include "subsystems/Climb.h"


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

constexpr int kCopilotControllerPort = 1;
constexpr int kSwerveControllerPort = 0;

constexpr double kDeadband = 0.08;

constexpr int kStrafeAxis = frc::XboxController::Axis::kLeftX;
constexpr int kForwardAxis = frc::XboxController::Axis::kLeftY;
constexpr int kRotationAxis = frc::XboxController::Axis::kRightX;
constexpr int kFieldRelativeButton = frc::XboxController::Button::kRightBumper;

constexpr int kIntakeGroundPOV = 90;
constexpr int kIntakeAMPPOV = 0;
constexpr int kIntakeShooterPOV = 270;

}  // namespace OperatorConstants

namespace FieldConstants
{

constexpr frc::Pose2d feeder_station{{625_in, 12_in}, -80_deg};

}

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

 public:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_copilotController{
      OperatorConstants::kCopilotControllerPort};

  frc2::CommandXboxController m_swerveController{
      OperatorConstants::kSwerveControllerPort};

  // The robot's subsystems are defined here...
  
  Shooter m_shooter;
  Drivetrain m_swerve;
  Intake m_intake;
  Climb m_climb;

  // Global Dashboard Items
  frc::Mechanism2d m_mech_sideview{4, 3};  // scaled to feet

public:
  void ConfigureBindings();
  void ConfigureDashboard();
};