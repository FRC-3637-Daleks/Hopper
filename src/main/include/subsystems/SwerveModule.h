#pragma once

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
#include <units/moment_of_inertia.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <numbers>

namespace ModuleConstants {

constexpr double NeutralDeadBand = 0.01;
constexpr double kDriveMotorCurrentLimit = 50; // Up to 80 A is okay
constexpr double kSteerMotorCurrentLimit = 50; // An educated guess.
constexpr auto kCurrentLimitPeriod =
    0.04_s; // Can exceed limit for 40ms seconds

// Best defense against current
constexpr double kMotorRampRate = 0.2; // Seconds from neutral to full output.

constexpr auto kWheelDiameterIdeal = 3.6_in;
constexpr double kWheelAdjust =
    1.0671 * 0.923; // Depricated after new wheel measurements
constexpr auto kWheelDiameter = kWheelDiameterIdeal;

constexpr double kDriveEncoderReduction = 6.75; // reduction in drive motor
constexpr auto kDriveEncoderDistancePerRevolution =
    kWheelDiameter * std::numbers::pi / kDriveEncoderReduction;
constexpr auto kWheelMoment = .015_kg_sq_m;

constexpr auto kDistanceToRotations = kDriveEncoderDistancePerRevolution / 1_tr;

constexpr double kSteerGearReduction = 150.0 / 7.0;
constexpr auto kSteerMoment = 0.005_kg_sq_m;

// Values measured with the drivetrain suspended.
constexpr auto kPhysicalMaxSpeed = 15.7_fps;
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
 * its wheel will be driven at the specified velocity in the specified
 * direction. The Drivetrain subsystem makes use of SwerveModule objects so that
 * it doesn't need to deal with directly commanding each motor.
 */
class SwerveModule {
public:
  // The ctor of the SwerveModule class.
  SwerveModule(const std::string name, const int driveMotorId,
               const int steerMotorId, const int absoluteEncoderId,
               const PIDCoefficients driveMotorPIDCoefficients,
               const PIDCoefficients steerMotorPIDCoefficients);

  // Need to define destructor to make simulation code compile
  ~SwerveModule();

  // IMPORTANT: Need to refresh signals once per loop
  void RefreshSignals();

  // This one is even more efficient than RefreshSignals as it groups ALL
  // swerve module signals into a single call
  template <typename... T> static void RefreshAllSignals(T &...modules);

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

  void CoastMode(bool coast);

  void SetEncoderOffset();

  void ZeroAbsEncoders();

  void SyncEncoders();

  // Commands the module to accelerate to a certain velocity and take on a
  // certain heading.
  void SetDesiredState(const frc::SwerveModuleState &state);

  // Sends the current swerve module state to the SmartDashboard.
  void UpdateDashboard();

  // Run physics simulation and update the hardware
  void SimulationPeriodic();

private:
  // Returns the absolute position of the steer motor in radians
  units::radian_t GetAbsoluteEncoderPosition();

  const std::string m_name; // Useful to identify the module.

  ctre::phoenix6::hardware::TalonFX m_driveMotor;

  ctre::phoenix6::hardware::TalonFX m_steerMotor;

  // Keeps track of the module heading between power cycles.
  ctre::phoenix6::hardware::CANcoder m_absoluteEncoder;

private: // signal objects to cache
  ctre::phoenix6::StatusSignal<units::angle::turn_t> m_drivePosition;
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t>
      m_driveVelocity;
  ctre::phoenix6::StatusSignal<units::angle::turn_t> m_steerPosition;
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t>
      m_steerVelocity;

private:
  friend class SwerveModuleSim;
  std::unique_ptr<SwerveModuleSim> m_sim_state;
};

// Template method must be defined in .h
template <typename... T> void SwerveModule::RefreshAllSignals(T &...modules) {
  // This passes all 4N signals to one call to RefreshAll
  ctre::phoenix6::BaseStatusSignal::RefreshAll(
      modules.m_drivePosition..., modules.m_driveVelocity...,
      modules.m_steerPosition..., modules.m_steerVelocity...);
}