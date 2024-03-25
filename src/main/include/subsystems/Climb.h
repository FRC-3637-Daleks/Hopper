// sparkflexmotorcontrollers

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>
#include <frc/system/plant/DCMotor.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <memory>
#include <numbers>

namespace ClimbConstants {
// Guess values to make sim work
constexpr auto kWindowMotor = frc::DCMotor{12_V, 70_inlb, 24_A, 5_A, 100_rpm};
constexpr double kClimbGearReduction = 72.0 / 18.0;
constexpr auto kClimbMass = 100_lb; // Weight of the Robot.
constexpr auto kClimbDrumRadius = 1.504_in;
constexpr auto kClimbStartingHeight = 0_in;
constexpr auto kClimbMaxHeight = 3_in;

constexpr int kClimbMotorPort = 13;

constexpr double kDistancePerRevolution =
    kClimbDrumRadius * std::numbers::pi / 1_in; // Guess value to make sim work
constexpr double kClimbMotorCPR = 1023 * kClimbGearReduction;
constexpr double kMotorEncoderDistancePerCount =
    kDistancePerRevolution / kClimbMotorCPR;

constexpr int kTimeoutMs = 20; // in ms.
constexpr int kPIDLoopIdx = 0;
}; // namespace ClimbConstants

class ClimbSimulation;

class Climb : public frc2::SubsystemBase {
public:
  Climb();
  ~Climb();

  void SimulationPeriodic() override;

  frc2::CommandPtr ExtendClimb();
  frc2::CommandPtr RetractClimb();
  frc2::CommandPtr StopClimb();
  frc2::CommandPtr ClimbCommand(std::function<double()> input);

  WPI_TalonSRX m_climbMotor{ClimbConstants::kClimbMotorPort};

private:
  friend class ClimbSimulation;
  std::unique_ptr<ClimbSimulation> m_sim_state;
};
