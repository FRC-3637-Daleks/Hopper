// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
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

}

class Intake : public frc2::SubsystemBase {
 public:


  Intake();


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

};
