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

// namespace ClimbConstants {
//     constexpr int kClimbMotorPort1 = 10;
//     constexpr int kClimbMotorPort2 = 12;
// }

class Climb : public frc2::SubsystemBase {
public:
    frc2::CommandPtr ExtendClimb();
    frc2::CommandPtr RetractClimb();
    frc2::CommandPtr StopClimb();

private:
// need to know climb motor port
    WPI_TalonSRX m_climbMotor{1};
    frc::DigitalInput m_climbBottom{2};
    frc::DigitalInput m_climbTop{3};

};
