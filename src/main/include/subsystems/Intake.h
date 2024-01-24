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

namespace IntakeConstants {
    constexpr int kIntakeMotorPort = 13;
    constexpr int kFlywheelMotorPort = 14;

    //arbitary & determined through testing
    constexpr int IntakeArmIntakePos = 1;
    constexpr int IntakeArmAMP = 2;
    constexpr int IntakeArmSpeaker = 3;


}
class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  
  frc2::CommandPtr IntakeCommand();

  //Not done yet, would move arm to specified position
  frc2::CommandPtr IntakeArmIntake(int IntakeArmIntakePos);
  frc2::CommandPtr IntakeArmAMP(int IntakeArmAMP);
  frc2::CommandPtr IntakeArmSpeaker(int IntakeArmSpeaker);

  //made for testing 
  void IntakeOn();
  void IntakeOff();

  //(still for testing) This is assuming the arm to move the other park of the intake will use a moter
  void IntakeRotateUp();
  void IntakeRotateDown();
  void IntakeRotateStop();

 private:
  
  rev::CANSparkFlex intake{IntakeConstants::kIntakeMotorPort, rev::CANSparkFlex::MotorType::kBrushless};
  
  //Assuming we are using a moter for the Intake Arm
  rev::CANSparkFlex rotate{IntakeConstants::kIntakeMotorPort, rev::CANSparkFlex::MotorType::kBrushless};

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

  frc::DigitalInput IntakeBreakBeam{0};
  bool hasBeenBroken = false; //Should only be used for Intake (not arm)
};
