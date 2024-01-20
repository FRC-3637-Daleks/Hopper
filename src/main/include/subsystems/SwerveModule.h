#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/AnalogInput.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/moment_of_inertia.h>

#include <memory>
#include <numbers>


namespace ModuleConstants {
constexpr double kDriveMotorCurrentLimit = 50; // Up to 80 A is okay.
constexpr double kSteerMotorCurrentLimit = 20; // An educated guess.

constexpr double kMotorRampRate = 0.5; // Seconds from neutral to full output.

constexpr auto kWheelDiameter = 4_in;
constexpr auto kDriveEncoderReduction = 6.75_tr;  // reduction in drive motor
constexpr double kDriveEncoderCPR = 4096.0;
constexpr auto kDriveEncoderDistancePerRevolution =
    kWheelDiameter * std::numbers::pi / kDriveEncoderReduction.value();
constexpr auto kWheelMoment = 0.015_kg_sq_m;

constexpr auto kDistanceToRotations = std::numbers::pi * ModuleConstants::kWheelDiameter / ModuleConstants::kDriveEncoderReduction;

constexpr double kSteerGearReduction = 150.0/7.0;
constexpr double kSteerEncoderCPR = 4096;
constexpr auto kSteerEncoderDistancePerCount =
    2_rad * std::numbers::pi / kSteerEncoderCPR; // Radians per encoder count.
constexpr auto kSteerMoment = 0.005_kg_sq_m;

// Values measured with the drivetrain suspended.
constexpr auto kPhysicalMaxSpeed = 16.5_fps;
constexpr auto kPhysicalMaxAngularSpeed = 180_rpm;
} // namespace ModuleConstants

// forward declaration
class SwerveModuleSim;

struct PIDCoefficients {
  double kP, kI, kD, kFF, kIz;
};

/**
 * The SwerveModule helper class consists of a steer motor and a drive motor
 * (both Falcon 500s/Krakens/Talon FXs).
 * Additionally, there is an absolute encoder (CANCoder) which regardless of
 * start position, reports the exact heading of the wheels.
 * Each module can be commanded to a certain state, that is, 
 * its wheel will be driven at the specified velocity in the specified direction.
 * The Drivetrain subsystem makes use of SwerveModule objects so that it doesn't
 * need to deal with directly commanding each motor.
 */
class SwerveModule {
public:
  // The ctor of the SwerveModule class.
  SwerveModule(const std::string name, const int driveMotorId,
               const int steerMotorId, const int absoluteEncoderId,
               const double absoluteEncoderOffset,
               const PIDCoefficients driveMotorPIDCoefficients,
               const PIDCoefficients steerMotorPIDCoefficients);

  // Need to define destructor to make simulation code compile
  ~SwerveModule();

  // Returns the meters driven based on encoder reading.
  units::meter_t GetModuleDistance();

  // Returns the velocity of the module in m/s.
  units::meters_per_second_t GetModuleVelocity();

  // Returns the module heading in the scope [-180,180] degrees.
  frc::Rotation2d GetModuleHeading();

  // Combines GetModuleDistance() and GetModuleHeading().
  frc::SwerveModulePosition GetPosition();

  // Combines GetModuleVelocity() and GetModuleHeading().
  frc::SwerveModuleState GetState();

  // Commands the module to accelerate to a certain velocity and take on a
  // certain heading.
  void SetDesiredState(const frc::SwerveModuleState &state);

  // Sends the current swerve module state to the SmartDashboard.
  void UpdateDashboard();

  // Run physics simulation and update the hardware
  // void SimulationPeriodic();

private:
  // Converts m/s to rpm for the drive velocity setpoint.
  double ToTalonVelocity(units::meters_per_second_t speed);

  // Converts an angle to Talon sensor units for the steer position setpoint.
  double ToTalonAngle(const frc::Rotation2d &rotation);

  // Returns the absolute position of the steer motor in radians
  units::radian_t GetAbsoluteEncoderPosition();

  const std::string m_name; // Useful to identify the module.

  ctre::phoenix6::hardware::TalonFX m_driveMotor;

  ctre::phoenix6::hardware::TalonFX m_steerMotor;

  // Keeps track of the module heading between power cycles.
  ctre::phoenix6::hardware::CANcoder m_absoluteEncoder;

private:
  friend class SwerveModuleSim;
  std::unique_ptr<SwerveModuleSim> m_sim_state;
};
