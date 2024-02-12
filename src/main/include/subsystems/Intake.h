// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/moment_of_inertia.h>
#include <units/torque.h>
#include <units/angle.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/DigitalInput.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/WaitUntilCommand.h>

#include <rev/CANSparkFlex.h>

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>

#include <rev/CANSparkLowLevel.h>

#include <ctre/Phoenix.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

#include <memory>
#include <numbers>

namespace IntakeConstants {
    //Moter IDs
    constexpr int kIntakeMotorPort = 13;
    constexpr int kArmlMotorPort = 14;

    //limit switch
    constexpr int kLimitSwitchIntakePort = 0;

    //breakbeam
    constexpr int kBreakbeamPort = 1;

    //From documetation: output value is in encoder ticks or an analog value, 
    //depending on the sensor
    constexpr int IntakeArmIntakePos = 1;
    constexpr int IntakeArmAMPPos = 2;
    constexpr int IntakeArmSpeakerPos = 3;

    constexpr bool kBeamBroken = true;
    constexpr bool kBeamClear = false;

    //something to do with the type of PID loop
    constexpr int kPIDLoopIdx = 0;

    //timeout for the PID loop
    constexpr int kTimeoutMs = 30;

    //pid configurations
    constexpr int kF = 0.0;
    constexpr int kP = 0.0;
    constexpr int kI = 0.0;
    constexpr int kD = 0.0;

    //margin of error for detecting if arm is in specific spot
    constexpr int kAllowableMarginOfError = 1;

    //voltage for funtions (i dide't even have to use auto)
    constexpr units::voltage::volt_t kOffVoltage = 0.0_V;
    constexpr units::voltage::volt_t kIntakeVoltage = 1.0_V;
    constexpr units::voltage::volt_t kShooterVoltage = 0.5_V;
    constexpr units::voltage::volt_t kAMPVoltage = 1.0_V;
    
    //physical characteristics
    constexpr auto kWheelMoment = 0.001_kg_sq_m;
    constexpr auto kWindowMotor = frc::DCMotor{12_V, 70_inlb, 24_A, 5_A, 100_rpm};
    constexpr auto kArmMass = 25_lb;
    constexpr auto kArmRadius = 13_in;
    constexpr auto kWheelDiameter = 1.5_in;  //< Verify this
    constexpr auto kWheelCircum = kWheelDiameter*std::numbers::pi/1_tr;
    constexpr double kArmGearing = 4.0;
    //you can play with the leading constant to get the dynamics you want
    constexpr auto kArmMoment = 0.5*kArmMass*kArmRadius*kArmRadius;
    constexpr bool kGravityCompensation = false;  // true if there's a gas spring

    // modeling 0 as intake horizontal in front of robot, and positive angle is counterclockwise
    constexpr auto kMinAngle = -30_deg;
    constexpr auto kMaxAngle = 160_deg;

    // TODO: MEASURE THESE
    constexpr int kArmSensorFullExtend = 10;  // corresponds to kMinAngle
    constexpr int kArmSensorFullRetract = 1000;  // corresponds to kMaxAngle
    constexpr auto kAngleToSensor = 
      (kArmSensorFullRetract - kArmSensorFullExtend) /
      (kMaxAngle - kMinAngle);
    constexpr auto kIntakeLength = 13.0_in;
    constexpr auto kIntakeSensorPosition = kIntakeLength - 1.0_in;
    
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

  void InitVisualization(frc::Mechanism2d* mech);
  void UpdateVisualization();

  /** Changes the direction of the moter
    * Makes the moter spin backwards (spitting game peice out)
    * Makes the moter spin forwards (intaking game peice)
  */
  void InvertIntake();
  void VertIntake();

  /** Simeple on off for intake
   * Turns on intake spinning forward for intaking game peice
   * Turns off intake for stopping intake
  */
  void OnIntake();
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

  /** Gets the state of Limit switches and break beams for intake
  * Gets the state of the limit switch for the intake
  * Gets the state of the break beam for the intake
  */
  bool GetStateLimitSwitchIntake();  
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

 private:

  /**
   * The motor used to run the intake.
  */
  rev::CANSparkFlex m_intake{IntakeConstants::kIntakeMotorPort, rev::CANSparkFlex::MotorType::kBrushless};

  /**
   * The motor used to run the intake arm. Is setup with a PID
  */
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_arm{IntakeConstants::kArmlMotorPort};

  /**
   * The goal position of the arm used for some functions
  */
  int goal;

  /** Defines some digital inputs
   * Defines limitswitch used on the intake
   * Defines breakbeam used on the intake
  */
  frc::DigitalInput m_limitSwitchIntake{IntakeConstants::kLimitSwitchIntakePort};
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
  void SimulateNotePickup();
};
