// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkFlex.h>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <frc/simulation/DCMotorSim.h>

#include <wpi/interpolating_map.h>

#include <memory>
#include <numbers>

namespace ShooterConstants {

// Constant values for getAnglePivot
constexpr auto kOffset = 0.0_deg;
constexpr int kLegOffset = 0.0;
// constexpr auto kPivotEncoderDistancePerCount = 0.1_deg;
constexpr auto kEncoderCPR = 0.0;
constexpr auto kGearReduction = 0.0;
constexpr bool kEncoderReversed = true;

constexpr int kPivotMotorPort = 14;
constexpr int kFlywheelLeadMotorPort = 16;
constexpr int kFlywheelFollowMotorPort = 18;

// PID Loop something
constexpr int kPIDLoopIdx = 0;

constexpr int kTimeoutMs = 20; // in ms.

// Guess values. Need accurate measurements

constexpr double kPivotEncoderReduction = (double)1 / 4;
constexpr double kPivotEncoderCPR =
    kPivotEncoderReduction * 28; // CPR is 4 counts/cycle * 7 cycles/revolution.
constexpr auto kPivotEncoderDistancePerCount =
    2_rad * std::numbers::pi / kPivotEncoderCPR; // Radians per encoder count.

// Guess values for Pivot PID. Need to calculate feed forward
constexpr double kPPivot = 5.0;
constexpr double kIPivot = 0.0;
constexpr double kDPivot = 0.0;
constexpr double kFPivot = 0.0;

// Physical Constants for Simulation
constexpr auto kWheelMass = 1_kg;
constexpr auto kWheelRadius = 2_in;
constexpr auto kWheelMoment = 0.5 * kWheelMass * kWheelRadius * kWheelRadius;

constexpr auto kWindowMotor = frc::DCMotor{12_V, 70_inlb, 24_A, 5_A, 100_rpm};
constexpr auto kArmGearing = 10;
constexpr auto kArmMass = 15_lb;
constexpr auto kArmRadius = 10_in;
constexpr auto kArmMoment = 0.5 * kArmMass * kArmRadius * kArmRadius;
/**Minimum angle(retracted)*/
constexpr auto kMinAngle = 0_deg;
/**Maximum angle(extended)*/
constexpr auto kMaxAngle = 80_deg;
constexpr auto kMinAimSensor = 935;
constexpr auto kMaxAimSensor = 51;
constexpr auto kMinIdeal = 920;
constexpr auto kMaxIdeal = 388;
constexpr auto kAngleToSensor =
    (kMaxAimSensor - kMinAimSensor) / (kMaxAngle - kMinAngle);
/**Note speed when exiting shooter (approximate)*/
constexpr auto kNoteVelocity = 15.7_mps;
constexpr auto kAmpShotAngle = 537;
constexpr auto kAmpShotPower = 0.21;
} // namespace ShooterConstants

// forward declaration
class ShooterSimulation;

class Shooter : public frc2::SubsystemBase {
public:
  Shooter();
  ~Shooter();

  void Periodic() override;
  /** Initializes the visualization in sim*/
  void InitVisualization(frc::Mechanism2d *mech);
  /** Updates the visualization in sim*/
  void UpdateVisualization();

  void SimulationPeriodic() override;

  // const PIDCoefficients m_pivotPIDCoefficients;

  /**Shoots ring
   * -Sets Shooter wheel motors to 1.00_volts
   */
  void RunShootMotor();
  /**Stops Shooter wheels*/
  void StopShootMotor();
  /** Runs Pivot motor at 1.00_volts*/
  void RunPivotMotor();
  /** Stops Pivot Motor*/
  void StopPivotMotor();
  /**Used to controll Shooter angle with joystick
   *  -Move Shooter angle to preset height(double in encoder ticks).*/
  void SetPivotMotor(double encoderPosition);
  /** Gets the current angle Shooter Pivot is at*/
  units::radian_t GetAnglePivot();
  /** Passes a distance (found using Apriltags & odometry),
   *  returns the angle needed to score into speaker from said distance
   */
  units::degree_t DistanceToAngle(units::foot_t distance);
  /** Finds the error in value returned by DistanceToAngle*/
  units::degree_t DistanceToAngleError(units::foot_t distance,
                                       units::radian_t angle);

  units::degree_t DistanceToAngleBinarySearch(units::foot_t distance);

  double distance_adjustment(units::feet_per_second_t robot_velocity,
                             units::foot_t distance, units::radian_t thetaf);

  units::foot_t
  DistanceAdjustmentBinarySearch(units::feet_per_second_t robot_velocity,
                                 units::foot_t distance,
                                 units::feet_per_second_t perp_velocity);

  double ToTalonUnits(const frc::Rotation2d &rotation);
  /**Passes the speed for the flywheels (const double encoder ticks) and
   * the velocity the pivot motor should move at.
   */
  frc2::CommandPtr ShooterVelocityCommand(
      std::function<double()> flywheelInput,
      std::function<units::angular_velocity::degrees_per_second_t()>
          pivotVelocity);
  // frc2::CommandPtr ShooterCommand(std::function<double()> flywheelInput,
  // std::function<units::degree_t()> pivotAngle);
  /**Passes the speed for the flywheel (const double encoder ticks) and
   * then uses the calculateDistance function to return the pivot angle
   */
  frc2::CommandPtr
  ShooterCommand(std::function<double()> flywheelInput,
                 std::function<units::meter_t()> calculateDistance);
  /**Passes the speed for the flywheel (const double encoder ticks) and
   * then uses the calculateDistance function to return the pivot angle.
   *
   * Also accounts for the forward and strafe velocity of the robot,
   * compensating for it.
   */
  frc2::CommandPtr ShooterVelocityDistanceCommand(
      std::function<double()> flywheelInput,
      std::function<units::meter_t()> calculateDistance,
      std::function<units::feet_per_second_t()> fwd_velocity,
      std::function<units::feet_per_second_t()> strafe_velocity);
  /** Sets the flywheel to speed determined by controller input*/
  frc2::CommandPtr FlywheelCommand(std::function<double()> controllerInput);
  /**Sets Shooter pivot angle to the value returned by a command*/
  frc2::CommandPtr
  PivotAngleCommand(std::function<units::degree_t()> pivotAngle);
  /**Finds the pivot angle needed to shoot on speaker by passing a distance in*/
  frc2::CommandPtr
  PivotAngleDistanceCommand(std::function<units::meter_t()> distance);

  frc2::CommandPtr AmpShot();
  //   frc2::CommandPtr
  //   AutoSpeakerFlywheelSpeed(std::function<units::meter_t()> distance);
  /**Finds the pivot angle needed to shoot on speaker by passing a distance in
   *
   * Also accounts for the forward and strafe velocity of the robot,
   * compensating for it.
   */

  frc2::CommandPtr PivotAngleVelocityDistanceCommand(
      std::function<units::foot_t()> distance,
      std::function<units::feet_per_second_t()> fwd_velocity,
      std::function<units::feet_per_second_t()> strafe_velocity);

  // initializes Lead + Follow motors (makes motors run in parallel)
  const int leadDeviceID = 1, followDeviceID = 2;

  rev::CANSparkFlex m_leadMotor{ShooterConstants::kFlywheelLeadMotorPort,
                                rev::CANSparkFlex::MotorType::kBrushless};
  rev::CANSparkFlex m_followMotor{ShooterConstants::kFlywheelFollowMotorPort,
                                  rev::CANSparkFlex::MotorType::kBrushless};

  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_pivot{
      ShooterConstants::kPivotMotorPort};

  units::degree_t m_goal;

  wpi::interpolating_map<units::meter_t, units::degree_t> m_map;

private:
  frc::MechanismLigament2d *m_mech_pivot, *m_mech_pivot_goal,
      *m_mech_mm_setpoint;

  // SIMULATION
  std::unique_ptr<ShooterSimulation> m_sim_state;
};
