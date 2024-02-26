// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/DigitalInput.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkFlex.h>

#include <ctre/Phoenix.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

#include <units/moment_of_inertia.h>
#include <units/torque.h>
#include <units/angle.h>

#include <memory>
#include <numbers>

namespace IntakeConstants {
    //Moter IDs
    constexpr int kIntakeMotorPort = 17;
    constexpr int kArmMotorPort = 15;

    //limit switch

    //breakbeam
    constexpr int kBreakbeamPort = 0;

    //From documetation: output value is in encoder ticks or an analog value, 
    //depending on the sensor
    constexpr int IntakeArmIntakePos = 955; // -
    constexpr int IntakeArmAMPPos = 640; // Needs to be 59.4 deg. After we get information on encoder offsets, actual value can be determined.
    constexpr int IntakeArmSpeakerPos = 440;

    constexpr bool kBeamBroken = true;
    constexpr bool kBeamClear = false;

    //something to do with the type of PID loop
    constexpr int kPIDLoopIdx = 0;

    //timeout for the PID loop
    constexpr int kTimeoutMs = 30;

    //pid configurations
    constexpr int kF = 0.0;
    constexpr int kP = 20.0;
    constexpr int kI = 0.0;
    constexpr int kD = 0.0;

    //consts for conversion
    constexpr int totalEncoders = 4096;

    //margin of error for detecting if arm is in specific spot
    constexpr int kAllowableMarginOfError = 10;

    //voltage for funtions (i dide't even have to use auto)
    constexpr units::voltage::volt_t kOffVoltage = 0.0_V;
    constexpr units::voltage::volt_t kIntakeVoltage = 1.0_V;
    constexpr units::voltage::volt_t kShooterVoltage = 0.5_V;
    constexpr units::voltage::volt_t kAMPVoltage = 1.0_V;
    
    //physical characteristics
    constexpr auto kWheelMoment = 0.001_kg_sq_m;
    constexpr auto kWindowMotor = frc::DCMotor{12_V, 70_inlb, 24_A, 5_A, 100_rpm};
    constexpr auto kArmMass = 12_lb;
    constexpr auto kArmRadius = 13_in;
    constexpr auto kWheelDiameter = 1.5_in;  //< Verify this
    constexpr auto kWheelCircum = kWheelDiameter*std::numbers::pi/1_tr;
    constexpr double kArmGearing = 4.0;
    //you can play with the leading constant to get the dynamics you want
    constexpr auto kArmMoment = 0.5*kArmMass*kArmRadius*kArmRadius;
    constexpr bool kGravityCompensation = true;  // true if there's a gas spring

    // modeling 0 as intake horizontal in front of robot, and positive angle is counterclockwise
    constexpr auto kMinAngle = -15.4_deg;
    constexpr auto kMaxAngle = 145_deg;

    // TODO: MEASURE THESE
    constexpr int kArmSensorFullExtend = 1020;  // corresponds to kMinAngle //amp angle = 1.365 radians
    constexpr int kArmSensorFullRetract = 430;  // corresponds to kMaxAngle
    constexpr auto kAngleToSensor = 
      (kArmSensorFullRetract - kArmSensorFullExtend) /
      (kMaxAngle - kMinAngle);
    constexpr auto kIntakeLength = 13.0_in;
    constexpr auto kIntakeSensorPosition = 0.5_in;
    
    constexpr auto sensorToAngle(int sensor)
    {return (sensor - kArmSensorFullExtend)/kAngleToSensor + kMinAngle;}

    constexpr auto angleToSensor(units::degree_t angle)
    {return (angle - kMinAngle)*kAngleToSensor + kArmSensorFullExtend;}

}

class IntakeSimulation;  // forward declaration

class Intake : public frc2::SubsystemBase {
 public:


  Intake();
  ~Intake();

  void Periodic() override;
  void SimulationPeriodic() override;

  /**
   * Sets the arm position to the intake position
   * runs intake until break beam or limit switch is hit
   * also stops when button is released
  */
  frc2::CommandPtr IntakeRing(); 

  /**
   * Sets the arm to the AMP position, 
   * when the button is down it shoots the ring
   * stops when the button is let go
  */
  frc2::CommandPtr ShootOnAMP();

  /**
   * Sets he arm to the Shooter position
   * Passes the ring to the shooter
   * Stops when:
   * a. Break beam on the shooter is tripped (will need get command from shooter class)
   * b. the button is released
  */
  frc2::CommandPtr OutputToShooter();

  /**
   * Set intake to spin forwards and take in a game piece
  */
  frc2::CommandPtr IntakeIn();

  frc2::CommandPtr AutoIntake();

  /**
   * Set intake to spin backwards to spit out a game piece
  */
  frc2::CommandPtr IntakeOut();

  // Keep intake Idle if no buttons are pressed
  frc2::CommandPtr IdleIntakeCommand();
  void InitVisualization(frc::Mechanism2d* mech);
  void UpdateVisualization();


  /** Simeple on off for intake
   * Turns on intake spinning forward for intaking game peice
   * Turns off intake for stopping intake
  */
  void IntakeForward();
  void IntakeBackward();
  void OffIntake();

  /**
  * Outputs the intake to the shooter
  * Outputs (and shoots) the intake to the AMP
  */
  void OutputShooterIntake();
  void OutputAMPIntake();

  /** Moves arm to specified position, position specified in constants
  * Moves the intake arm to the AMP position
  * Moves the intake arm to the Speaker position
  * Moves the intake arm to the Intake position
  */
  void IntakeArmAMP();
  void IntakeArmSpeaker();
  void IntakeArmIntake();

  frc2::CommandPtr IntakeArmAMPCommand(bool wait = false);
  frc2::CommandPtr IntakeArmSpeakerCommand(bool wait = false);
  frc2::CommandPtr IntakeArmIntakeCommand(bool wait = false);

  /** Gets the state of Limit switches and break beams for intake
  * Gets the state of the limit switch for the intake
  * Gets the state of the break beam for the intake
  */

  bool GetStateBreakBeamIntake();

  /** 
  * Gets the difference between were the arm is going and were it is 
  */
  int GetArmDiffrence();

  /** sees if the arm is at a specific position, 
  has a margin of error defined in constants
  * Sees if the arm is at the amp position
  * Sees if the arm is at the speaker position
  * Sees if the arm is at the intake position
  */
  bool IsAtAMP();
  bool IsAtSpeaker();
  bool IsAtIntake();

  /**
   * The motor used to run the intake.
  */
  rev::CANSparkFlex m_intake{IntakeConstants::kIntakeMotorPort, rev::CANSparkFlex::MotorType::kBrushless};

  /**
   * The motor used to run the intake arm. Is setup with a PID
  */
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_arm{IntakeConstants::kArmMotorPort};

  /**
   * The goal position of the arm used for some functions
  */
  int m_goal;

  /** Defines some digital inputs
   * Defines limitswitch used on the intake
   * Defines breakbeam used on the intake
  */
  frc::DigitalInput m_breakbeam{IntakeConstants::kBreakbeamPort};

private:
  frc::MechanismRoot2d* m_mech_root;
  frc::MechanismLigament2d *m_mech_arm;
  frc::MechanismLigament2d *m_mech_arm_goal;
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
