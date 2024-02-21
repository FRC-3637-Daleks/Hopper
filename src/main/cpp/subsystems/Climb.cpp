//Breakbeams + possibly absolute encoders

// ClimbSubsystem.cpp
#include "subsystems/Climb.h"

#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/simulation/ElevatorSim.h>
#include <frc/simulation/DIOSim.h>

class ClimbSimulation {
public:
    ClimbSimulation(Climb &climb):
        m_motorSim{climb.m_climbMotor.GetSimCollection()},
        m_climbModel{
            ClimbConstants::kWindowMotor,
            ClimbConstants::kClimbGearReduction,
            ClimbConstants::kClimbMass,
            ClimbConstants::kClimbDrumRadius,
            ClimbConstants::kClimbStartingHeight,
            ClimbConstants::kClimbMaxHeight,
            true,
            ClimbConstants::kClimbStartingHeight
        }
    {}
public:
    ctre::phoenix::motorcontrol::TalonSRXSimCollection m_motorSim;

    frc::sim::ElevatorSim m_climbModel;
};

Climb::Climb():
    m_sim_state{new ClimbSimulation(*this)} {

  m_climbMotor.ConfigFactoryDefault();

  m_climbMotor.SetInverted(false);
//   m_climbMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

  m_climbMotor.SetSelectedSensorPosition(0, ClimbConstants::kPIDLoopIdx, ClimbConstants::kTimeoutMs);
  m_climbMotor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::Analog/*idk if correct*/, ClimbConstants::kPIDLoopIdx, ClimbConstants::kTimeoutMs);
  m_climbMotor.SetSensorPhase(false);

  //Configure Minimum and Maximum outputs.
  m_climbMotor.ConfigNominalOutputForward(0.0, ClimbConstants::kTimeoutMs);
  m_climbMotor.ConfigNominalOutputReverse(0.0, ClimbConstants::kTimeoutMs);
  m_climbMotor.ConfigPeakOutputForward(1.0, ClimbConstants::kTimeoutMs);
  m_climbMotor.ConfigPeakOutputReverse(-1.0, ClimbConstants::kTimeoutMs);
}

Climb::~Climb() {}

void Climb::SimulationPeriodic() {
    if(!m_sim_state) return;

    m_sim_state->m_motorSim.SetBusVoltage(frc::RobotController::GetBatteryVoltage().value());
    m_sim_state->m_climbModel.SetInputVoltage(m_sim_state->m_motorSim.GetMotorOutputLeadVoltage() * 1_V);

    m_sim_state->m_climbModel.Update(20_ms);

    m_sim_state->m_motorSim.SetAnalogPosition(m_sim_state->m_climbModel.GetPosition() / ClimbConstants::kMotorEncoderDistancePerCount / 1_m);
    m_sim_state->m_motorSim.SetAnalogVelocity(m_sim_state->m_climbModel.GetVelocity() / ClimbConstants::kMotorEncoderDistancePerCount / 1_mps * 100_ms / 1_ms);

    m_sim_state->m_motorSim.SetSupplyCurrent(
        m_sim_state->m_climbModel.GetCurrentDraw().value()
    );

    if(m_sim_state->m_climbModel.HasHitUpperLimit())
        m_sim_state->m_motorSim.SetLimitFwd(true);
    else
        m_sim_state->m_motorSim.SetLimitFwd(false);

    if(m_sim_state->m_climbModel.HasHitLowerLimit())
        m_sim_state->m_motorSim.SetLimitRev(true);
    else
        m_sim_state->m_motorSim.SetLimitRev(false);

    frc::SmartDashboard::PutNumber("Climb/sim/Climb Position (m)", m_sim_state->m_climbModel.GetPosition().value());
    frc::SmartDashboard::PutNumber("Climb/sim/Climb Velocity (mps)", m_sim_state->m_climbModel.GetVelocity().value());
}

frc2::CommandPtr Climb::ClimbCommand(std::function<double()> input) {
    return frc2::cmd::Run(
        [this, input] {
            frc::SmartDashboard::PutNumber("Climb/Climb Input: ", input());
            m_climbMotor.Set(input());
        }, {this}
    );
}

frc2::CommandPtr Climb::ExtendClimb() {
    frc::SmartDashboard::PutNumber("Climb Input: ", 1);
    return frc2::cmd::StartEnd([this] {
            m_climbMotor.Set(1);
        }, [this] () {
             m_climbMotor.Set(0);
        }, {this});
}

frc2::CommandPtr Climb::RetractClimb() {
    frc::SmartDashboard::PutNumber("Climb Input: ", -1);
    return frc2::cmd::StartEnd([this] {
            m_climbMotor.Set(-1);
        }, [this] () {
             m_climbMotor.Set(0);
        }, {this});       
}


frc2::CommandPtr Climb::StopClimb() {
   return Run([this] {m_climbMotor.Set(0);});
}

