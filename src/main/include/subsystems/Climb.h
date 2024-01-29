//sparkflexmotorcontrollers

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>

// namespace ClimbConstants {
//     constexpr int kClimbMotorPort1 = 10;
//     constexpr int kClimbMotorPort2 = 12;
// }

class Climb : public frc2::SubsystemBase {
public:
    void ExtendClimb();
    void RetractClimb();
    void StopClimb();

private:
// need to know climb motor port
    WPI_TalonSRX m_climbMotor{1};
};
