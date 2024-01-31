#pragma once
// using namespace units::literals;
// #include <units/units.h>

#include <AHRS.h>
#include <cmath>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/PowerDistribution.h>
#include <hal/SimDevice.h>
#include <hal/simulation/SimDeviceData.h>

#include <frc/controller/ProfiledPIDController.h>

#include <memory>
#include <numbers>

#include "SwerveModule.h"

namespace DriveConstants {
constexpr bool kGyroReversed = true;

constexpr auto kMaxSpeed = 12_fps;
constexpr auto kMaxTeleopSpeed = 15_fps;
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

constexpr double kPTurn = 0.0605;  
constexpr double kITurn = 0.001; 
constexpr double kDTurn = 0.03;   

constexpr auto kMaxTurnRate = 3 * std::numbers::pi * 1_rad_per_s;
constexpr auto kMaxTurnAcceleration = 3 * std::numbers::pi * 1_rad_per_s_sq;




constexpr double kPDistance = 2;
constexpr auto kDistanceTolerance = 7_cm;

constexpr double kPLeftStraight = 0.2;
constexpr double kPRightStraight = 0.2;

constexpr auto kTurnTolerance = 3_deg;
constexpr auto kTurnRateTolerance = 1_deg_per_s;


// Swerve Constants
constexpr auto kTrackWidth =
    25_in; // Distance between centers of right and left wheels.
constexpr auto kWheelBase =
    25_in; // Distance between centers of front and back wheels.

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
  10.009775171065494, 0.0, 0.05004887585532747, 0, 0
};
constexpr struct PIDCoefficients kRearLeftSteerMotorPIDCoefficients {
  10.009775171065494, 0.0, 0.05004887585532747, 0, 0
};
constexpr struct PIDCoefficients kFrontRightSteerMotorPIDCoefficients {
  10.009775171065494, 0.0, 0.05004887585532747, 0, 0
};
constexpr struct PIDCoefficients kRearRightSteerMotorPIDCoefficients {
  10.009775171065494, 0.0, 0.05004887585532747, 0, 0
};


} // namespace DriveConstants

// Forward Declaration
class DrivetrainSimulation;

/**
 * The Drivetrain subsystem contains four swerve modules and a gyroscope. The
 * Drivetrain can be driven at a certain speed represented as a 2-dimensional
 * translation on the field. This translation can be commanded either relative
 * to the front of the robot, or relative to a certain absolute position. These
 * two modes are called "robot relative" and "field relative" respectively. The
 * drivetrain can also be commanded to rotate at a given speed independent of
 * its translation.
 */
class Drivetrain : public frc2::SubsystemBase {
public:
  // The ctor of the Drivetrain subsystem.
  Drivetrain();

  // Need to define destructor to make simulation code compile
  ~Drivetrain();

  // Updates the odometer and SmartDashboard.
  void Periodic() override;

  // Executes the simulation
  void SimulationPeriodic() override;

  // Drives the robot at the given translational and rotational speed. Forward
  // is front-back movement, strafe is left-right movement. If fieldRelative is
  // set, the translation is relative to the field instead of the robot heading.
  void Drive(units::meters_per_second_t forwardSpeed,
             units::meters_per_second_t strafeSpeed,
             units::radians_per_second_t angularSpeed, bool fieldRelative);

  // Sets the state of each swerve module.
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  // Returns the heading of the robot.
  frc::Rotation2d GetHeading();

  // Zeroes the robot heading.
  void ZeroHeading();

  // Returns the rotational velocity of the robot in degrees per second.
  units::degrees_per_second_t GetTurnRate();

  // Returns the robot heading and translation as a Pose2d.
  frc::Pose2d GetPose();

  // Resets the odometry using the given a field-relative pose using current
  // gyro angle.
  void ResetOdometry(const frc::Pose2d &pose);

  // The kinematics model for a swerve drive. Once given the positions of each
  // module with the robot's center as the origin, the SwerveDriveKinematics
  // class will convert between chassis speeds (translation and rotation) and
  // module states (velocity and angle).
  frc::SwerveDriveKinematics<4> kDriveKinematics{
      frc::Translation2d(DriveConstants::kWheelBase / 2,
                         DriveConstants::kTrackWidth / 2),
      frc::Translation2d(DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2),
      frc::Translation2d(-DriveConstants::kWheelBase / 2,
                         DriveConstants::kTrackWidth / 2),
      frc::Translation2d(-DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2)};

  // Display useful information on Shuffleboard.
  void UpdateDashboard();

  // Drive the robot with swerve controls.
  frc2::CommandPtr
  SwerveCommand(std::function<units::meters_per_second_t()> forward,
                std::function<units::meters_per_second_t()> strafe,
                std::function<units::revolutions_per_minute_t()> rot);

  // Drive the robot with field-relative swerve controls.
  frc2::CommandPtr SwerveCommandFieldRelative(
      std::function<units::meters_per_second_t()> forward,
      std::function<units::meters_per_second_t()> strafe,
      std::function<units::revolutions_per_minute_t()> rot);

  // Drive the robot to pose.
  // frc2::CommandPtr DriveToPoseCommand(frc::Pose2d targetPose);

  // Check if the robot has reached a pose.
  bool IsFinished(frc::Pose2d targetPose);

  // Returns a command that zeroes the robot heading.
  frc2::CommandPtr ZeroHeadingCommand();

  // Returns a command that stops the robot.
  frc2::CommandPtr BrakeCommand();

  // Add Vision Pose to SwerveDrivePoseEstimator.
  void AddVisionPoseEstimate(frc::Pose2d pose, units::second_t timestamp);

  frc2::CommandPtr TurnToAngleCommand(units::degree_t angle);

  frc2::CommandPtr ZTargetPoseCommand(frc::Pose2d pose, 
    std::function<units::meters_per_second_t()> forward,
    std::function<units::meters_per_second_t()> strafe);

private:
  SwerveModule m_frontLeft;
  SwerveModule m_rearLeft;
  SwerveModule m_frontRight;
  SwerveModule m_rearRight;

  AHRS m_gyro;

  // Odometer for tracking the robot's position on the field.
  // frc::SwerveDriveOdometry<4> m_odometry;

  // Pose Estimator for estimating the robot's position on the field.
  frc::SwerveDrivePoseEstimator<4> m_poseEstimator;

  // Field widget for Shuffleboard.
  frc::Field2d m_field;

  frc::PowerDistribution m_pdh{15,
                               frc::PowerDistribution::ModuleType::kRev};

  frc::ProfiledPIDController<units::degree> m_turnPID{DriveConstants::kPTurn, DriveConstants::kITurn, DriveConstants::kDTurn, {DriveConstants::kMaxTurnRate, DriveConstants::kMaxTurnAcceleration}};
  
private:
  friend class DrivetrainSimulation;
  std::unique_ptr<DrivetrainSimulation> m_sim_state;
};