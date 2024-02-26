// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DigitalInput.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkFlex.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>

#include <units/mass.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/moment_of_inertia.h>

#include <frc/simulation/DCMotorSim.h>

#include <memory>
#include <numbers>


namespace ShooterConstants {

  //Constant values for getAnglePivot
    constexpr auto kOffset = 0.0_deg;
    constexpr int kLegOffset = 0.0;
    //constexpr auto kPivotEncoderDistancePerCount = 0.1_deg;
    constexpr auto kEncoderCPR = 0.0;
    constexpr auto kGearReduction = 0.0;
    constexpr bool kEncoderReversed = true;

  constexpr int kPivotMotorPort = 14;
  constexpr int kFlywheelLeadMotorPort = 16;
  constexpr int kFlywheelFollowMotorPort = 18;

 //PID Loop something
    constexpr int kPIDLoopIdx = 0;

    constexpr int kTimeoutMs = 20; // in ms.

    // Guess values. Need accurate measurements

    constexpr double kPivotEncoderReduction =
    (double) 1 / 4;
constexpr double kPivotEncoderCPR =
    kPivotEncoderReduction * 28; // CPR is 4 counts/cycle * 7 cycles/revolution.
constexpr auto kPivotEncoderDistancePerCount =
    2_rad * std::numbers::pi / kPivotEncoderCPR; // Radians per encoder count.

    //Guess values for Pivot PID. Need to calculate feed forward
    constexpr double kPPivot = 5.0;
    constexpr double kIPivot = 0.0;
    constexpr double kDPivot = 0.0;
    constexpr double kFPivot = 0.0;

    // Physical Constants for Simulation
    constexpr auto kWheelMass = 1_kg;
    constexpr auto kWheelRadius = 2_in;
    constexpr auto kWheelMoment = 0.5*kWheelMass*kWheelRadius*kWheelRadius;

    constexpr auto kWindowMotor = frc::DCMotor{12_V, 70_inlb, 24_A, 5_A, 100_rpm};
    constexpr auto kArmGearing = 10;
    constexpr auto kArmMass = 15_lb;
    constexpr auto kArmRadius = 10_in;
    constexpr auto kArmMoment = 0.5*kArmMass*kArmRadius*kArmRadius;

    constexpr auto kMinAngle = 0_deg;
    constexpr auto kMaxAngle = 80_deg;
    constexpr auto kMinAimSensor = 935;
    constexpr auto kMaxAimSensor = 51;
    constexpr auto kAngleToSensor = (kMaxAimSensor - kMinAimSensor)/(kMaxAngle - kMinAngle);
}

// forward declaration
class ShooterSimulation;

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();
  ~Shooter();

void Periodic() override;

  void InitVisualization(frc::Mechanism2d* mech);
  void UpdateVisualization();

  void SimulationPeriodic() override;
  
  // const PIDCoefficients m_pivotPIDCoefficients;

  //Runs and Stops Motors - basic voids
  void RunShootMotor(); 

  void StopShootMotor();

  void RunTalonMotor();

  void StopTalonMotor();

  void SetPivotMotor(double encoderPosition);
  
  units::radian_t GetAnglePivot(); 

  units::degree_t DistanceToAngle(units::foot_t distance);

  double ToTalonUnits(const frc::Rotation2d &rotation);

<<<<<<< HEAD
  // frc2::CommandPtr ShooterCommand(std::function<double()> flywheelInput, std::function<units::degree_t()> pivotAngle);

  frc2::CommandPtr ShooterCommand(std::function<double()> flywheelInput, std::function<units::angular_velocity::degrees_per_second_t()> pivotVelocity);

=======
  frc2::CommandPtr ShooterVelocityCommand(std::function<double()> flywheelInput, std::function<units::angular_velocity::degrees_per_second_t()> pivotVelocity);
  // frc2::CommandPtr ShooterCommand(std::function<double()> flywheelInput, std::function<units::degree_t()> pivotAngle);

>>>>>>> origin/visvam-wip
  frc2::CommandPtr ShooterCommand(std::function<double()> flywheelInput, std::function<units::meter_t()> calculateDistance);
  
  frc2::CommandPtr FlywheelCommand(std::function<double()> controllerInput);

  frc2::CommandPtr PivotAngleCommand(std::function<units::degree_t()> pivotAngle);

<<<<<<< HEAD
  frc2::CommandPtr PivotAngleDistanceCommand(units::meter_t distance);

  frc2::CommandPtr AimSubwoofer(units::meter_t distance);
=======
  frc2::CommandPtr PivotAngleDistanceCommand(std::function<units::meter_t()> distance);
>>>>>>> origin/visvam-wip

 //initializes Lead + Follow motors (makes motors run in parallel) 
  const int leadDeviceID = 1, followDeviceID = 2;

  rev::CANSparkFlex m_leadMotor{ShooterConstants::kFlywheelLeadMotorPort, rev::CANSparkFlex::MotorType::kBrushless};
  rev::CANSparkFlex m_followMotor{ShooterConstants::kFlywheelFollowMotorPort, rev::CANSparkFlex::MotorType::kBrushless};
 

  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_pivot{ShooterConstants::kPivotMotorPort};
  
  units::degree_t m_goal;
  
 private:
  
  frc::MechanismLigament2d *m_mech_pivot, *m_mech_pivot_goal;

  // SIMULATION 
  std::unique_ptr<ShooterSimulation> m_sim_state;
};
