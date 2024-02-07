// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/mass.h>
#include <units/length.h>
#include <units/moment_of_inertia.h>

#include <frc/DigitalInput.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <rev/CANSparkFlex.h>
#include <ctre/Phoenix.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/simulation/DCMotorSim.h>
#include <cmath>

#include <memory>



namespace ShooterConstants {
    constexpr int kShooterLeadMotorPort = 15;
    constexpr int kShooterFollowerMotorPort = 16;
    constexpr int kAimMotorPort = 17;
    constexpr int kPIDLoopIdx = 0;

    constexpr auto kTimeoutMs = 50;


    // Physical Constants for Simulation
    constexpr auto kWheelMass = 1_kg;
    constexpr auto kWheelRadius = 2_in;
    constexpr auto kWheelMoment = 0.5*kWheelMass*kWheelRadius*kWheelRadius;

    constexpr auto kWindowMotor = frc::DCMotor{12_V, 70_inlb, 24_A, 5_A, 100_rpm};
    constexpr auto kArmGearing = 10;
    constexpr auto kArmMass = 15_lb;
    constexpr auto kArmRadius = 10_in;
    constexpr auto kArmMoment = 0.5*kArmMass*kArmRadius*kArmRadius;

    constexpr auto kMinAngle = 10_deg;
    constexpr auto kMaxAngle = 60_deg;
    constexpr auto kMinAimSensor = 10;
    constexpr auto kMaxAimSensor = 1000;
    constexpr auto kAngleToSensor = (kMaxAimSensor - kMinAimSensor)/(kMaxAngle - kMinAngle);
}

// forward declaration
class ShooterSimulation;

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();
  ~Shooter();

  void SimulationPeriodic() override;
  
  //const PIDCoefficients m_pivotPIDCoefficients;

  //Runs and Stops Motors - basic voids
  void runShootMotor(); 

  void stopShootMotor();

  void runTalonMotor();

  void stopTalonMotor();

  void Periodic() override; 

  frc2::CommandPtr IntakeCommand();

  frc2::CommandPtr FlywheelCommand( double controllerInput);

  rev::CANSparkFlex m_leadMotor{ShooterConstants::kShooterLeadMotorPort,rev::CANSparkFlex::MotorType::kBrushless};
  rev::CANSparkFlex m_followMotor{ShooterConstants::kShooterFollowerMotorPort,rev::CANSparkFlex::MotorType::kBrushless};
 

  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_pivot{ShooterConstants::kAimMotorPort};



 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc::DigitalInput m_intakeBreakBeam{0};
  
  frc::DigitalInput m_flywheelBreakBeam{1};

private:
  // SIMULATION 
  std::unique_ptr<ShooterSimulation> m_sim_state;
};
