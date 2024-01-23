// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>


Shooter::Shooter() {
  // Implementation of subsystem constructor goes here.

//SmartDashboard 
frc::SmartDashboard::PutNumber("motor power", m_motor.Get()); 
}

//Runs both motors
void Shooter::runMotor(){

// Needs arguments to work between power cycles!!
// Resets config perameters
  m_followMotor.RestoreFactoryDefaults();
  m_leadMotor.RestoreFactoryDefaults();

//Motors following + leading

  m_followMotor.Follow(m_leadMotor);
//Starts
   m_leadMotor.Set(1);
   
}

void Shooter::stopMotor(){

m_followMotor.Follow(m_leadMotor);

  m_leadMotor.StopMotor();

}

frc2::CommandPtr Shooter::IntakeCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  // return StartEnd([this] { intake.Set(0.5); }, 
  //                 [this] { intake.Set(0.0); });
  return frc2::cmd::Idle();
}

frc2::CommandPtr Shooter::FlywheelCommand( double controllerInput ) {
  return Run([this, &controllerInput] 
             { m_motor.Set(controllerInput); });
}
