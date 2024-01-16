// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter() {
  // Implementation of subsystem constructor goes here.
}

frc2::CommandPtr Shooter::IntakeCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return StartEnd([this] { intake.Set(0.5); }, 
                  [this] { intake.Set(0.0); });
}

frc2::CommandPtr Shooter::FlywheelCommand( double controllerInput ) {
  return Run([this, &controllerInput] 
             { flywheel.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::Velocity, controllerInput); });
}
