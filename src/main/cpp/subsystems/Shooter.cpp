// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>


Shooter::Shooter() {
// Implementation of subsystem constructor goes here.
// Needs arguments to work between power cycles!!
// Resets config perameters
  m_followMotor.RestoreFactoryDefaults();
  m_leadMotor.RestoreFactoryDefaults();

  m_pivot.ConfigFactoryDefault();

  m_pivot.SetSelectedSensorPosition(0, ShooterConstants::kPIDLoopIdx, ShooterConstants::kTimeoutMs);

  m_pivot.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::Analog, ShooterConstants::kPIDLoopIdx, ShooterConstants::kTimeoutMs);

  m_pivot.SetSensorPhase(true);

  m_pivot.ConfigNominalOutputForward(0, ShooterConstants::kTimeoutMs);
  m_pivot.ConfigNominalOutputReverse(0, ShooterConstants::kTimeoutMs);
  m_pivot.ConfigPeakOutputForward(1.0, ShooterConstants::kTimeoutMs);
  m_pivot.ConfigPeakOutputReverse(1.0, ShooterConstants::kTimeoutMs);
  
  m_pivot.Config_kF(ShooterConstants::kPIDLoopIdx, 0.0, ShooterConstants::kTimeoutMs);
  m_pivot.Config_kP(ShooterConstants::kPIDLoopIdx, 0.0, ShooterConstants::kTimeoutMs);
  m_pivot.Config_kI(ShooterConstants::kPIDLoopIdx, 0.0, ShooterConstants::kTimeoutMs);
  m_pivot.Config_kD(ShooterConstants::kPIDLoopIdx, 0.0, ShooterConstants::kTimeoutMs);
//Motors following + leading

  m_followMotor.Follow(m_leadMotor);
  
//Configure for use 

  


}

//Runs both shooting motors
void Shooter::RunShootMotor(){

//Starts
   m_leadMotor.SetVoltage(1.0_V);
   
}

//Stops both shooting motors
void Shooter::StopShootMotor(){

//Stops
  m_leadMotor.StopMotor();

}

//Runs pivoting motor
void Shooter::RunTalonMotor(){
//Runs
m_pivot.SetVoltage(1.0_V);

}

//Stop pivoting motor
void Shooter::StopShootMotor(){
//Stops
 m_pivot.StopMotor();

}

void Shooter::Periodic(){

//SmartDashboard 
frc::SmartDashboard::PutNumber("motor power", m_leadMotor.Get()); 

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
             { m_leadMotor.Set(controllerInput); });
}


