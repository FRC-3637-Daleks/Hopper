// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

<<<<<<< Updated upstream
=======
#include "Constants.h"
#include <rev/CANSparkFlex.h>
>>>>>>> Stashed changes
#include <frc/DigitalInput.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/WaitUntilCommand.h>

<<<<<<< Updated upstream
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>


namespace ShooterConstants {
    constexpr int kIntakeMotorPort = 13;
    constexpr int kFlywheelMotorPort = 14;
}

=======
>>>>>>> Stashed changes
class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();

  /**
   * Run the intake motor.
   */


 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
    rev::CANSparkFlex flywheelMotor{ShooterConstants::kFlywheelMotorPort, rev::CANSparkFlex::MotorType::kBrushless};
    rev::SparkPIDController flywheelPID{flywheelMotor.GetPIDController()};

};
