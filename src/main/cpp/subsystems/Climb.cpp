// ClimbSubsystem.cpp

#include "subsystems/Climb.h"


void Climb::ExtendClimb() {
    m_climbMotor.Set(1);
}

void Climb::RetractClimb() {
    m_climbMotor.Set(-1);
}

void Climb::StopClimb() {
    m_climbMotor.Set(0);
}