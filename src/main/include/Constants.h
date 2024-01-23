// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/ArmFeedforward.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/XboxController.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/moment_of_inertia.h>


#include <numbers>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

struct PIDCoefficients {
  double kP, kI, kD, kFF, kIz;
};

namespace OperatorConstants {

constexpr int kDriverControllerPort = 0;
constexpr int kSwerveControllerPort = 0;

constexpr double kDeadband = 0.08;

constexpr int kStrafeAxis = frc::XboxController::Axis::kLeftX;
constexpr int kForwardAxis = frc::XboxController::Axis::kLeftY;
constexpr int kRotationAxis = frc::XboxController::Axis::kRightX;
constexpr int kFieldRelativeButton = frc::XboxController::Button::kRightBumper;

}  // namespace OperatorConstants

namespace ShooterConstants {
    constexpr int kIntakeMotorPort = 13;
    constexpr int kFlywheelMotorPort = 14;
}

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


namespace ModuleConstants {
constexpr double kDriveMotorCurrentLimit = 50; // Up to 80 A is okay.
constexpr double kSteerMotorCurrentLimit = 20; // An educated guess.

constexpr double kMotorRampRate = 0.5; // Seconds from neutral to full output.

constexpr auto kWheelDiameter = 4_in;
constexpr double kDriveEncoderReduction = 6.75;  // reduction in drive motor
constexpr double kDriveEncoderCPR = 4096;
constexpr auto kDriveEncoderDistancePerRevolution =
    kWheelDiameter * std::numbers::pi / kDriveEncoderReduction;
constexpr auto kWheelMoment = 0.015_kg_sq_m;

constexpr double kSteerGearReduction = 150.0/7.0;
constexpr double kSteerEncoderCPR = 4096;
constexpr auto kSteerEncoderDistancePerCount =
    2_rad * std::numbers::pi / kSteerEncoderCPR; // Radians per encoder count.
constexpr auto kSteerMoment = 0.005_kg_sq_m;

// Values measured with the drivetrain suspended.
constexpr auto kPhysicalMaxSpeed = 16.5_fps;
constexpr auto kPhysicalMaxAngularSpeed = 180_rpm;
} // namespace ModuleConstants

namespace DriveConstants {
constexpr auto kTrackWidth = 24_in;


constexpr bool kGyroReversed = true;

constexpr auto kMaxSpeed = 18_fps;
constexpr auto kArcadeMaxSpeed = 10_fps;
constexpr auto kPreciseSpeed = 2_fps;

// PID coefficients for closed-loop control of velocity.
constexpr double kFDriveSpeed = 0.0656;
constexpr double kPDriveSpeed = 0.1;
constexpr double kIDriveSpeed = 0.000;
constexpr double kDDriveSpeed = 0;
constexpr double kIzDriveSpeed = 1000;

constexpr double kIBrake = 0.0001;

// NOTE: Guess value!
constexpr double kPTurn = 2.2;
constexpr double kPDistance = 2;
constexpr auto kDistanceTolerance = 7_cm;

constexpr double kPLeftStraight = 0.2;
constexpr double kPRightStraight = 0.2;

constexpr auto kTurnTolerance = 3_deg;
constexpr auto kTurnRateTolerance = 1_deg_per_s;

constexpr auto kMaxTurnRate = 1 * std::numbers::pi * 1_rad_per_s;
constexpr auto kMaxTurnAcceleration = 1 * std::numbers::pi * 1_rad_per_s_sq;

// Swerve Constants (NEED TO INTEGRATE)

// left out as this variable are repeated above
// constexpr auto kTrackWidth =
//    20.25_in; // Distance between centers of right and left wheels.
constexpr auto kWheelBase = 20_in; // Distance between centers of front and back wheels.

 
      
    

constexpr int kFrontLeftDriveMotorId = 1;
constexpr int kRearLeftDriveMotorId = 3;
constexpr int kFrontRightDriveMotorId = 5;
constexpr int kRearRightDriveMotorId = 7;

constexpr int kFrontLeftSteerMotorId = 2;
constexpr int kRearLeftSteerMotorId = 4;
constexpr int kFrontRightSteerMotorId = 6;
constexpr int kRearRightSteerMotorId = 8;

constexpr int kFrontLeftAbsoluteEncoderChannel = 9;
constexpr int kRearLeftAbsoluteEncoderChannel = 10;
constexpr int kFrontRightAbsoluteEncoderChannel = 11;
constexpr int kRearRightAbsoluteEncoderChannel = 12;

// Absolute encoder reading when modules are facing forward.
constexpr double kFrontLeftAbsoluteEncoderOffset = 3.15246;
constexpr double kRearLeftAbsoluteEncoderOffset = -2.25482;
constexpr double kFrontRightAbsoluteEncoderOffset = -2.03871;
constexpr double kRearRightAbsoluteEncoderOffset = 1.377484;

// XXX Roughly estimated values, needs to be properly tuned.
constexpr struct PIDCoefficients kFrontLeftDriveMotorPIDCoefficients {
  0, 0, 0, 0, 0
};
constexpr struct PIDCoefficients kRearLeftDriveMotorPIDCoefficients {
  0, 0, 0, 0, 0
};
constexpr struct PIDCoefficients kFrontRightDriveMotorPIDCoefficients {
  0, 0, 0, 0, 0
};
constexpr struct PIDCoefficients kRearRightDriveMotorPIDCoefficients {
  0, 0, 0, 0, 0
};

constexpr struct PIDCoefficients kFrontLeftSteerMotorPIDCoefficients {
  5.0, 0.0, 25, 0, 0
};
constexpr struct PIDCoefficients kRearLeftSteerMotorPIDCoefficients {
  5.0, 0.0, 25, 0, 0
};
constexpr struct PIDCoefficients kFrontRightSteerMotorPIDCoefficients {
  5.0, 0.0, 25, 0, 0
};
constexpr struct PIDCoefficients kRearRightSteerMotorPIDCoefficients {
  5.0, 0.0, 25, 0, 0
};

constexpr auto kMaxTeleopSpeed = 15_fps;
// constexpr auto kPreciseSpeed = 2_fps; // left out because it already exists
// above

} // namespace DriveConstants
