// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DigitalInput.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <rev/CANSparkFlex.h>
#include <ctre/Phoenix.h>
#include <frc/drive/DifferentialDrive.h>


#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/moment_of_inertia.h>

#include <cmath>
#include <numbers>

namespace ShooterConstants {
    constexpr int kPivotMotorPort = 13;
    constexpr int kFlywheelLeaderMotorPort = 14;
    constexpr int kFlywheelFollowMotorPort = 0;

    constexpr int kTimeoutMs = 20; //in ms.
    constexpr int kPIDLoopIdx = 0; 

    // Guess values. Need accurate measurements

    constexpr double kPivotEncoderReduction =
    (double) 1 / 4;
constexpr double kPivotEncoderCPR =
    kPivotEncoderReduction * 28; // CPR is 4 counts/cycle * 7 cycles/revolution.
constexpr auto kPivotEncoderDistancePerCount =
    2_rad * std::numbers::pi / kPivotEncoderCPR; // Radians per encoder count.

    //Guess values for Pivot PID. Need to calculate feed forward
    constexpr double kPPivot = 1.0;
    constexpr double kIPivot = 0.0;
    constexpr double kDPivot = 0.0;
    constexpr double kFPivot = 0.0;
}

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();
  
  // const PIDCoefficients m_pivotPIDCoefficients;

  //Runs and Stops Motors - basic voids
  void RunShootMotor(); 

  void StopShootMotor();

  void RunTalonMotor();

  void StopTalonMotor();

  void SetPivotMotor(double encoderPosition);

  void Periodic() override; 

  units::degree_t DistanceToAngle(units::meter_t distance);

  double ToTalonUnits(const frc::Rotation2d &rotation);

  frc2::CommandPtr FlywheelCommand(double controllerInput);

  frc2::CommandPtr PivotAngleCommand(units::degree_t angle);

  frc2::CommandPtr PivotAngleDistanceCommand(units::meter_t distance);

 //Lead + Follow motors (makes motors run in parallel) what constructors?
  const int leadDeviceID = 1, followDeviceID = 2;

  rev::CANSparkFlex m_leadMotor{ShooterConstants::kFlywheelLeaderMotorPort, rev::CANSparkFlex::MotorType::kBrushless};
  rev::CANSparkFlex m_followMotor{ShooterConstants::kFlywheelFollowMotorPort, rev::CANSparkFlex::MotorType::kBrushless};
 

  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_pivot{ShooterConstants::kPivotMotorPort};



 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc::DigitalInput m_intakeBreakBeam{0};
  
  frc::DigitalInput m_flywheelBreakBeam{1};
  
};
