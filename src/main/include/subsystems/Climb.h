//sparkflexmotorcontrollers

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc/DigitalInput.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/moment_of_inertia.h>
#include <units/mass.h>

namespace ClimbConstants {
    // Guess values to make sim work
    constexpr double kClimbGearReduction = 1.0 / 4.0;
    constexpr auto kClimbMass = 50_kg;
    constexpr auto kClimbDrumRadius = 3_in;
    constexpr auto kClimbStartingHeight = 2_ft + 4_in;
    constexpr auto kClimbMaxHeight = 2_ft + 6_in;

    constexpr int kClimbMotorPort = 18;
    constexpr int kClimbBottomLimitSwitch = 5;
    constexpr int kClimbTopLimitSwitch = 6;

    constexpr double kDistancePerRevolution = 10; // Guess value to make sim work
    constexpr double kClimbMotorCPR = 1023 * kClimbGearReduction;
    constexpr double kMotorEncoderDistancePerCount =  kDistancePerRevolution / kClimbMotorCPR;
    
};

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
    frc::DigitalInput m_climbBottom{ClimbConstants::kClimbBottomLimitSwitch};
    frc::DigitalInput m_climbTop{ClimbConstants::kClimbTopLimitSwitch};
private:
    std::unique_ptr<ClimbSimulation> m_sim_state;
};
