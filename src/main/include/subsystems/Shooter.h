// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"

#include <frc/DigitalInput.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <rev/CANSparkFlex.h>
#include <frc/drive/DifferentialDrive.h>



class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();
  
  //Runs Motor? - basic void commands 
  void runMotor(); 
  void stopMotor();
  
  frc2::CommandPtr IntakeCommand();

  frc2::CommandPtr FlywheelCommand( double controllerInput);

 //Lead + Follow motors (makes motors run in parallel) what constructors?
  const int leadDeviceID = 1, followDeviceID = 2;

  rev::CANSparkFlex m_leadMotor{leadDeviceID,rev::CANSparkFlex::MotorType::kBrushless};
  rev::CANSparkFlex m_followMotor{followDeviceID,rev::CANSparkFlex::MotorType::kBrushless};
 


 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkFlex m_motor{ShooterConstants::kFlywheelMotorPort, rev::CANSparkFlex::MotorType::kBrushless};
  rev::CANSparkFlex m_motor{ShooterConstants::kFlywheelMotorPort, rev::CANSparkFlex::MotorType::kBrushless};
  frc::DigitalInput m_intakeBreakBeam{0};
  frc::DigitalInput m_flywheelBreakBeam{1};
};
