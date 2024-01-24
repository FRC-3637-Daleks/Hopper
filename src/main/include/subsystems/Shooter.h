// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DigitalInput.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/WaitUntilCommand.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>


namespace ShooterConstants {
    constexpr int kIntakeMotorPort = 13;
    constexpr int kFlywheelMotorPort = 14;
}

class Shooter {
 public:
  Shooter();

  /**
   * Run the intake motor.
   */
  frc2::CommandPtr IntakeCommand(double controllerInput);

  frc2::CommandPtr FlywheelCommand( double controllerInput);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX intake{ShooterConstants::kIntakeMotorPort};
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX flywheel{ShooterConstants::kFlywheelMotorPort};

  frc::DigitalInput m_intakeBreakBeam{0};
  frc::DigitalInput m_flywheelBreakBeam{1};
};