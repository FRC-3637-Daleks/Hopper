#pragma once
// using namespace units::literals;
// #include <units/units.h>

#include <AHRS.h>
#include <cmath>
#include <frc/PowerDistribution.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/controller/ProfiledPIDController.h>

#include <memory>
#include <numbers>

#include "SwerveModule.h"

namespace DriveConstants {
constexpr auto kMaxSpeed = 15.7_fps;
constexpr auto kMaxTeleopSpeed = 15.7_fps;

constexpr auto kMaxTurnRate = 1.5 * std::numbers::pi * 1_rad_per_s;
constexpr auto kMaxTurnAcceleration = 2 * std::numbers::pi * 1_rad_per_s_sq;

// NOTE: Guess value!

constexpr double kPTurn = 0.071; // 0.061
constexpr double kITurn = 0.00;  // 0.00
constexpr double kDTurn = 0.00;  // 0.0

// Swerve Constants
constexpr auto kTrackWidth =
    25_in; // Distance between centers of right and left wheels.
constexpr auto kWheelBase =
    25_in; // Distance between centers of front and back wheels.
// const auto kRadius = 19.5_in; // 19.5 inches
const auto kRadius = units::meter_t(std::sqrt(.91));

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
  0, 0.00, 0, 0, 0
};
constexpr struct PIDCoefficients kRearLeftDriveMotorPIDCoefficients {
  0, 0.00, 0, 0, 0
};
constexpr struct PIDCoefficients kFrontRightDriveMotorPIDCoefficients {
  0, 0.00, 0, 0, 0
};
constexpr struct PIDCoefficients kRearRightDriveMotorPIDCoefficients {
  0, 0.00, 0, 0, 0
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

constexpr double kS = 0.0545;

constexpr frc::Pose2d kSpeakerPose{0.14_m, 5.5222_m, 0_deg};
constexpr frc::Pose2d kAMPPose{1.812_m, 8.239_m, 0_deg};
constexpr frc::Pose2d kStagePose{4.869_m, 4.144_m, 0_deg};
constexpr frc::Pose2d kSourcePose{15.733_m, 0.410_m, 0_deg};

// estimation
constexpr auto kNoteVelocity = 50_fps;

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
             units::radians_per_second_t angularSpeed, bool fieldRelative,
             bool isRed);

  // Sets the state of each swerve module.
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  // Returns the heading of the robot.
  frc::Rotation2d GetHeading();

  frc::Rotation2d GetGyroHeading();

  // Zeroes the robot heading.
  void ZeroHeading();

  void ZeroAbsEncoders();

  void SetAbsEncoderOffset();

  void SyncEncoders();

  void CoastMode(bool coast);

  // Returns the rotational velocity of the robot in degrees per second.
  units::degrees_per_second_t GetTurnRate();

  // Returns the robot heading and translation as a Pose2d.
  frc::Pose2d GetPose();

  // Returns Current Chassis Speed
  frc::ChassisSpeeds GetSpeed();
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
  frc::Field2d &GetField() { return m_field; }

  // Drive the robot with swerve controls.
  frc2::CommandPtr
  SwerveCommand(std::function<units::meters_per_second_t()> forward,
                std::function<units::meters_per_second_t()> strafe,
                std::function<units::revolutions_per_minute_t()> rot);

  // Drive the robot with field-relative swerve controls.
  frc2::CommandPtr SwerveCommandFieldRelative(
      std::function<units::meters_per_second_t()> forward,
      std::function<units::meters_per_second_t()> strafe,
      std::function<units::revolutions_per_minute_t()> rot,
      std::function<bool()> isRed);

  frc2::CommandPtr
  SwerveSlowCommand(std::function<units::meters_per_second_t()> forward,
                    std::function<units::meters_per_second_t()> strafe,
                    std::function<units::revolutions_per_minute_t()> rot,
                    std::function<bool()> isRed);
  // Drive the robot to pose.
  // frc2::CommandPtr DriveToPoseCommand(frc::Pose2d targetPose);

  // Check if the robot has reached a pose.
  bool IsFinished(frc::Pose2d targetPose);

  // Returns a command that zeroes the robot heading.
  frc2::CommandPtr ZeroHeadingCommand();

  frc2::CommandPtr ZeroAbsEncodersCommand();

  frc2::CommandPtr SetAbsEncoderOffsetCommand();

  frc2::CommandPtr CoastModeCommand(bool coast);

  frc2::CommandPtr ConfigAbsEncoderCommand();

  // Returns a command that stops the robot.
  frc2::CommandPtr BrakeCommand();

  // Add Vision Pose to SwerveDrivePoseEstimator.
  void AddVisionPoseEstimate(frc::Pose2d pose, units::second_t timestamp,
                             wpi::array<double, 3U> visionMeasurementStdDevs);

  frc2::CommandPtr TurnToAngleCommand(units::degree_t angle);

  frc2::CommandPtr
  ZTargetPoseCommand(std::function<frc::Pose2d()> pose,
                     std::function<units::meters_per_second_t()> forward,
                     std::function<units::meters_per_second_t()> strafe,
                     bool shooterSide, std::function<bool()> isRed);

private:
  SwerveModule m_frontLeft;
  SwerveModule m_rearLeft;
  SwerveModule m_frontRight;
  SwerveModule m_rearRight;

  AHRS m_gyro;

  // Pose Estimator for estimating the robot's position on the field.
  frc::SwerveDrivePoseEstimator<4> m_poseEstimator;

  // Field widget for Shuffleboard.
  frc::Field2d m_field;

  frc::PowerDistribution m_pdh{25, frc::PowerDistribution::ModuleType::kRev};

  frc::ProfiledPIDController<units::degree> m_turnPID{
      DriveConstants::kPTurn,
      DriveConstants::kITurn,
      DriveConstants::kDTurn,
      {DriveConstants::kMaxTurnRate, DriveConstants::kMaxTurnAcceleration}};
  frc2::CommandPtr zeroEncodersCommand{ZeroAbsEncodersCommand()};

  frc::Pose2d m_zTarget;

  frc::Transform2d m_odometryCompensation{0_m, 0_m, 0_deg};

private:
  friend class DrivetrainSimulation;
  std::unique_ptr<DrivetrainSimulation> m_sim_state;
};