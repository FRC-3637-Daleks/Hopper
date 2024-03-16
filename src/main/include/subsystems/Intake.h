// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc2/command/ConditionalCommand.h"
#include <frc/DigitalInput.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkFlex.h>

#include <ctre/Phoenix.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

#include <units/angle.h>
#include <units/moment_of_inertia.h>
#include <units/torque.h>

#include <memory>
#include <numbers>

#include <frc2/command/ParallelCommandGroup.h>

namespace IntakeConstants {

// Moter IDs
constexpr int kIntakeMotorPort = 17;
constexpr int kArmMotorPort = 15;

// limit switch

// breakbeam
constexpr int kBreakbeamPort = 0;

// From documetation: output value is in encoder ticks or an analog value,
// depending on the sensor
constexpr int IntakeArmIntakePos = 995; // -
constexpr int IntakeArmAMPPos =
    660; // maybe make forward to catch the note if miss
constexpr int IntakeArmAMPVelocityPos = IntakeArmAMPPos - 10;

constexpr int IntakeArmSourceIntakePos = 625;
constexpr int IntakeArmSpeakerPos = 441;
// constexpr int IntakeArmPreAMPPos = 600;
constexpr int IntakeArmLetGoPos = 560; // Maybe make later

constexpr bool kBeamBroken = false;
constexpr bool kBeamClear = false;

// something to do with the type of PID loop
constexpr int kPIDLoopIdx = 0;

// timeout for the PID loop
constexpr int kTimeoutMs = 30;

constexpr double kNominalOutputFwd = 0.1;
constexpr double kNominalOutputRev = 0;
constexpr double kNeutralDeadband = 0.05;

// pid configurations
constexpr float kF = 4.0;
constexpr float kP = 3.0;
constexpr float kI = 0.0;
constexpr float kD = 0.0;

constexpr int totalEncoders = 4096;

// margin of error for detecting if arm is in specific spot
constexpr int kAllowableMarginOfError = 15;

// voltage for funtions (i dide't even have to use auto)
constexpr units::voltage::volt_t kOffVoltage = 0.0_V;
constexpr units::current::ampere_t kMaxCurrent = 20_A;

// physical characteristics
constexpr auto kWheelMoment = 1.0_kg_sq_m;
// constexpr auto kWindowMotor =
//    frc::DCMotor::Vex775Pro; // Hardcoded in m_armModel
constexpr auto kArmMass = 12_lb;
constexpr auto kArmRadius = 13_in;
constexpr auto kWheelDiameter = 1.5_in; //< Verify this
constexpr auto kWheelCircum = kWheelDiameter * std::numbers::pi / 1_tr;

constexpr double kArmGearing = 4.0 * 50;
// you can play with the leading constant to get the dynamics you want
constexpr auto kArmMoment = 1.0 * kArmMass * kArmRadius * kArmRadius;
constexpr bool kGravityCompensation = true; // true if there's a gas spring

// modeling 0 as intake horizontal in front of robot, and positive angle is
// counterclockwise
constexpr auto kMinAngle = -15.4_deg;
constexpr auto kMaxAngle = 145_deg;

// TODO: MEASURE THESE
constexpr int kArmSensorFullExtend =
    1020; // corresponds to kMinAngle //amp angle = 1.365 radians
constexpr int kArmSensorFullRetract = 400; // corresponds to kMaxAngle
constexpr auto kAngleToSensor =
    (kArmSensorFullRetract - kArmSensorFullExtend) / (kMaxAngle - kMinAngle);
constexpr auto kIntakeLength = 13.0_in;
constexpr auto kIntakeSensorPosition = 0.5_in;

constexpr auto sensorToAngle(int sensor) {
  return (sensor - kArmSensorFullExtend) / kAngleToSensor + kMinAngle;
}

constexpr auto angleToSensor(units::degree_t angle) {
  return (angle - kMinAngle) * kAngleToSensor + kArmSensorFullExtend;
}

} // namespace IntakeConstants

class IntakeSimulation; // forward declaration

class Intake : public frc2::SubsystemBase {
public:
  Intake();
  ~Intake();

  void Periodic() override;
  void SimulationPeriodic() override;

  void Emergency(double input);

  int PreviousSensorPosition = 0;
  /** Automaticaly intakes ring and goes to speaker pos
   * If doesent have ring
   * -Goes to ground position
   * -Spins intake until breakbeam is tripped
   * Goes to speaker position
   */
  frc2::CommandPtr IntakeRing();

  /** Automatically intakes ring from player station
   * Uses AMP pos as speaker pos (goes to pos)
   * AutoIntakes, no waiting for arm to get to position
   */
  frc2::CommandPtr IntakeFromPlayerStation();

  /** Shoots on the AMP when lined up
   * If has ring
   * -Goes to speaker position
   * -Goes to AMP position
   * -When passes specfic encoder point on the way to AMP, outputs
   */
  frc2::CommandPtr ShootOnAMP();

  /** Shoot to speaker
   * If has ring
   * -Goes to speaker position
   * -Outputs
   */
  frc2::CommandPtr OutputToShooter();

  /** ONLY FOR DRIVER CONTROL
   * Sets voltage (may cause exepected behavior) until button is let go, then
   * off
   */
  frc2::CommandPtr IntakeIn();

  /** Sets voltage to 0
   * uses a RunOnce
   */
  frc2::CommandPtr IntakeOff();

  /** Spits the ring out (for AMP speed) by setting voltage
   * Warning: may never stop
   */
  frc2::CommandPtr IntakeOut();

  /** Spins intake in until breakbeam is tripped
   * Spins intake
   * Until breakbeam is tripped
   * Then stops intake
   */
  frc2::CommandPtr AutoIntake();

  /** Outputs to speaker
   * Uses speaker voltage
   */
  frc2::CommandPtr IntakeOutSpeaker();

  /** Timed release for AMP
   * Turns the intake off
   * until the encoder position is greater than IntakeArmLetGoPos
   * Then outputs to the AMP
   * DEPRICATED: Use ShootOnAMPVoid (its a bit diffrent)
   */
  frc2::CommandPtr TimedRelease();

  /** Moves arm to AMP and outputs
   * Always moves arm to AMP position
   * if greater than goal position
   * shoots out at amp speed
   */
  void ShootOnAMPVoid();

  // Keep intake Idle if no buttons are pressed
  frc2::CommandPtr IdleIntakeCommand();
  void InitVisualization(frc::Mechanism2d *mech);
  void UpdateVisualization();

  // (voltage) Intake ring in
  void IntakeForward();

  // (voltage) Intake ring out (AMP Speed)
  void IntakeBackward();

  // (voltage) Intake ring out (Speaker Speed)
  void IntakeBackwardSpeaker();

  // (voltage) Stops intake
  void OffIntake();

  // Moves arm to AMP using motion magic (also sets goal (for visualization))
  void IntakeArmAMP();

  void IntakeArmAMPVelocity();

  // Moves arm to speaker using motion magic (also sets goal (for
  // visualization))
  void IntakeArmSpeaker();

  // Moves arm to intake using motion magic (also sets goal (for visualization))
  void IntakeArmIntake();

  // Moves arm to intake using motion magic (also sets goal (for visualization))
  void IntakeArmSource();

  // Checks if arm is at passed in position (goal != m_goal)
  bool IsAtWantedPosition(int goal);

  // Uses corresponding void function to move to AMP position, if wait is true,
  // waits for cmd to finish, if false does not wait
  frc2::CommandPtr IntakeArmAMPCommand(bool wait = false);

  // Uses corresponding void function to move to Source position, if wait is
  // true, waits for cmd to finish, if false does not wait
  frc2::CommandPtr IntakeArmSourceCommand(bool wait = false);

  frc2::CommandPtr IntakeArmAMPVelocityCommand(bool wait = false);
  // waits for cmd to finish, if false does not wait
  frc2::CommandPtr IntakeArmSpeakerCommand(bool wait = false);
  // Uses corresponding void function to move to  ground, if wait is true,
  // waits for cmd to finish, if false does not wait
  frc2::CommandPtr IntakeArmIntakeCommand(bool wait = false);

  // If the break beam is broken then it return true
  bool IsIntakeBreakBeamBroken();

  /** Gets diffrence between goal and current position
   * DEPRICATED: Use IsAtWantedPosition
   */
  int GetArmDiffrence();

  /** Sees if were at the named position
   * DEPRICATED: Use IsAtWantedPosition
   */
  bool IsAtAMP();
  bool IsAtSpeaker();
  bool IsAtIntake();

  // Moter for spinning the intake
  rev::CANSparkFlex m_intake{IntakeConstants::kIntakeMotorPort,
                             rev::CANSparkFlex::MotorType::kBrushless};

  // Moter for moving the arm
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_arm{
      IntakeConstants::kArmMotorPort};

  // Goal for the arm, display only (sim/drive station)
  int m_goal;

  // Breakbeam for detecting if a ring is in the intake
  frc::DigitalInput m_breakbeam{IntakeConstants::kBreakbeamPort};

private:
  frc::MechanismRoot2d *m_mech_root;
  frc::MechanismLigament2d *m_mech_arm;
  frc::MechanismLigament2d *m_mech_arm_goal;
  frc::MechanismLigament2d *m_mech_arm_mm_setpoint;
  frc::MechanismLigament2d *m_mech_spinner;
  frc::MechanismLigament2d *m_mech_note;

private:
  friend class IntakeSimulation;
  std::unique_ptr<IntakeSimulation> m_sim_state;

public:
  /* For simulation only. This allows the higher level simulation
   * to tell the intake simulation when a Note is at the spot where intake
   * will pick it up. (for example when robot is facing feeder station)
   */
  bool SimulateNotePickup();
};
