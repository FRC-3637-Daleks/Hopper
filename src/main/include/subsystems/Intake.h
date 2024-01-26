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

#include <rev/CANSparkMax.h>

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>

namespace IntakeConstants {
    //Moter IDs
    constexpr int kIntakeMotorPort = 13;
    constexpr int kArmlMotorPort = 14;

    //Position of the arm, most likely moter encoder positions
    constexpr int IntakeArmIntakePos = 1000;
    constexpr int IntakeArmAMPPos = 1500;
    constexpr int IntakeArmSpeakerPos = 2000;


}
class Intake : public frc2::SubsystemBase {
 public:

  /** Clears and sets up paramiters for PID, IZone, FF, and Output Range
   * Look at private for the defition of the values
   */
  Intake();

  /**Used existing intake functions to automatically stop ring after its secured
   * Button command is bound to should be heald down from time when trying 
     to pick up ring to sortly after ring is secured inside intake 
  */
  frc2::CommandPtr IntakeCommand();

  /**
   * Uses PID Controller to move arm to positions 
     as specified in IntakeConstants namespace
  */
  frc2::CommandPtr IntakeArmIntake();
  frc2::CommandPtr IntakeArmAMP();
  frc2::CommandPtr IntakeArmSpeaker();

  /**
   * Moves arm to specified position and outputs ring
  */
  frc2::CommandPtr IntakeAMPOutput();
  frc2::CommandPtr IntakeSpeakerOutput();

  /**
   * Basic funtion for turning moter on and off
  */
  frc2::CommandPtr IntakeOn();
  frc2::CommandPtr IntakeOff();

 private:

  /* My Idea of how the intake would look like with lazer
  ### Purpose: automatically stops wheels after driver intakes ring, driver just has to hold the button down ###

  * = rod for wheels
  - = break beam
  assume moter and gear placement
    ___________
      *    *   
               -  _________ 
    __*____*___   |_______| <- ring

    when beam is being broken, always spin, if beam has not been broek before & is not broken, spin
    ___________
    __*____*____   
    |__________|- <- ring with break beam in front inside intake
    __*____*___   

    when the beam has been broken and is not broken now, then the ring has been taken in and should be just on the edge of the beam
    when driver lets go of intake button, hasRing becomes false making it possible to pick up another beam
    when passing from intake to AMP or speaker, hasRing should not been touched as intake button will not being pressed
  */

  /**Defines intake moter
   * DeviceID:  Defined in namespace IntakeConstants
   * Type:    Type of moter that we are using (kBushless)
  */
  rev::CANSparkMax intake{IntakeConstants::kIntakeMotorPort, rev::CANSparkMax::MotorType::kBrushless};
  
  /** Defines Break beam for intake, used to detect ring, look at diagram above
   * DeviceID: the ID of the break beam
  */
  frc::DigitalInput IntakeBreakBeam{0};

  /**Used to know if we have a ring in the intake
   * Used only for IntakeCommand()
   * When this is true the beam has been broken before, 
   * If this is true and the break beam is clear then theres a ring in the intake
   * Is reset when driver let goes of intake button
  */
  bool hasBeenBroken = false; 

  //A lot of code for the Intake Arm is taken from:
//https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/C%2B%2B/Velocity%20PID%20Control/src/main/cpp/Robot.cpp
  
  /**Defines rotate moter (the one used for the arm part of the intake)
   * DeviceID:  Defined in namespace IntakeConstants
   * Type:    Type of moter that we are using (kBushless)
  */
  rev::CANSparkMax rotate{IntakeConstants::kArmlMotorPort, rev::CANSparkMax::MotorType::kBrushless};

  /**Setup for the PID, initialized in the constructor
   * Values gotten form example Github
   * kP = Proportional value 
   * kI = Integral value
   * kD = Derivative value
   * kIz = IZone limits error range where integral grain is active
   * kFF = FowardFeed value
   * kMinOutput/kMaxOutput = limit output range for controller
  */
  double kP = 0.1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;

  /** Binds m_pidController to rotate (i think)
   * Setup for the pid is found above, initialized in the constructor
  */
  rev::SparkPIDController m_pidController = rotate.GetPIDController();

};
