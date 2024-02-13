//Breakbeams + possibly absolute encoders

// ClimbSubsystem.cpp
#include "subsystems/Climb.h"

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/Commands.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/simulation/ElevatorSim.h>
#include <frc/simulation/DIOSim.h>

class ClimbSimulation {
public:
    ClimbSimulation(Climb &climb):
        m_motorSim{climb.m_climbMotor.GetSimCollection()},
        m_bottomLimitSwitchSim{climb.m_climbBottom},
        m_topLimitSwitchSim{climb.m_climbTop},
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

    frc::sim::DIOSim m_bottomLimitSwitchSim;
    frc::sim::DIOSim m_topLimitSwitchSim;

    frc::sim::ElevatorSim m_climbModel;
};

Climb::Climb():
    m_sim_state{new ClimbSimulation(*this)}
 {}

Climb::~Climb() {}

void Climb::SimulationPeriodic() {
    if(!m_sim_state) return;

    m_sim_state->m_motorSim.SetBusVoltage(frc::RobotController::GetBatteryVoltage().value());
    m_sim_state->m_climbModel.SetInputVoltage(m_sim_state->m_motorSim.GetMotorOutputLeadVoltage() * 1_V);

    m_sim_state->m_climbModel.Update(20_ms);

    m_sim_state->m_motorSim.SetAnalogPosition(m_sim_state->m_climbModel.GetPosition() / ClimbConstants::kMotorEncoderDistancePerCount / 1_m);
    m_sim_state->m_motorSim.SetAnalogVelocity(m_sim_state->m_climbModel.GetVelocity() / ClimbConstants::kMotorEncoderDistancePerCount / 1_mps * 100_ms / 1_ms);

    m_sim_state->m_motorSim.SetLimitFwd(
        m_sim_state->m_climbModel.HasHitUpperLimit()
    );

    m_sim_state->m_motorSim.SetLimitRev(
        m_sim_state->m_climbModel.HasHitLowerLimit()
    );

    m_sim_state->m_motorSim.SetSupplyCurrent(
        m_sim_state->m_climbModel.GetCurrentDraw().value()
    );

    if(m_sim_state->m_climbModel.HasHitUpperLimit())
        m_sim_state->m_topLimitSwitchSim.SetValue(true);
    else
        m_sim_state->m_topLimitSwitchSim.SetValue(false);

    if(m_sim_state->m_climbModel.HasHitLowerLimit())
        m_sim_state->m_bottomLimitSwitchSim.SetValue(true);
    else
        m_sim_state->m_bottomLimitSwitchSim.SetValue(false);

    frc::SmartDashboard::PutNumber("Climb/sim/Climb Position (m)", m_sim_state->m_climbModel.GetPosition().value());
    frc::SmartDashboard::PutNumber("Climb/sim/Climb Velocity (mps)", m_sim_state->m_climbModel.GetVelocity().value());
    frc::SmartDashboard::PutBoolean("Climb/sim/Top Limit ", m_sim_state->m_topLimitSwitchSim.GetValue());
    frc::SmartDashboard::PutBoolean("Climb/sim/Bottom Limit ", m_sim_state->m_bottomLimitSwitchSim.GetValue());
}

frc2::CommandPtr Climb::ClimbCommand(std::function<double()> input) {
    frc::SmartDashboard::PutNumber("Climb Input: ", input());
    frc::SmartDashboard::PutBoolean("Climb Top Limit: ", m_climbTop.Get());
    frc::SmartDashboard::PutBoolean("Climb Bottom Limit: ", m_climbBottom.Get());
    return frc2::cmd::RunEnd([this, input] {
            m_climbMotor.Set(input());
            frc::SmartDashboard::PutNumber("Climb Input: ", input());
        }, [this] () {
             m_climbMotor.Set(0);
             frc::SmartDashboard::PutBoolean("Climb Top Limit: ", m_climbTop.Get());
             frc::SmartDashboard::PutBoolean("Climb Bottom Limit: ", m_climbBottom.Get());
        }, {this})
        .Until([this, input] () -> bool {
            return ( 
            ((input() > 0.0)/*when going up*/ &&/*and*/ (m_climbTop.Get())/*max out*/) /*stop*/
            || /*or*/ ((input() < 0.0)/*when going down*/ &&/*and*/ (m_climbBottom.Get())/*hit bottom*/) /*stop*/
            );
        });
}

frc2::CommandPtr Climb::ExtendClimb() {
    //return Run([this] {m_climbMotor.Set(1);});

    frc::SmartDashboard::PutNumber("Climb Input: ", 1);
    frc::SmartDashboard::PutBoolean("Climb Top Limit: ", m_climbTop.Get());
    frc::SmartDashboard::PutBoolean("Climb Bottom Limit: ", m_climbBottom.Get());
    return frc2::cmd::RunEnd([this] {
            m_climbMotor.Set(1);
        }, [this] () {
             m_climbMotor.Set(0);
             frc::SmartDashboard::PutBoolean("Climb Top Limit: ", m_climbTop.Get());
             frc::SmartDashboard::PutBoolean("Climb Bottom Limit: ", m_climbBottom.Get());
        }, {this})
        .Until([this] () -> bool {
            return (m_climbBottom.Get() || m_climbTop.Get());
        });
    // return frc2::ConditionalCommand(
    //     frc2::cmd::Run([this] {m_climbMotor.Set(1);}),
    //     frc2::cmd::Run([this] {m_climbMotor.Set(0);}),
    //     [this] () -> bool {return (m_climbBottom.Get() && m_climbTop.Get());}
    // ).ToPtr();

}

frc2::CommandPtr Climb::RetractClimb() {
    //return Run([this] {m_climbMotor.Set(-1);});
    frc::SmartDashboard::PutNumber("Climb Input: ", -1);
    frc::SmartDashboard::PutBoolean("Climb Top Limit: ", m_climbTop.Get());
    frc::SmartDashboard::PutBoolean("Climb Bottom Limit: ", m_climbBottom.Get());
    return frc2::cmd::RunEnd([this] {
            m_climbMotor.Set(-1);
        }, [this] () {
             m_climbMotor.Set(0);
             frc::SmartDashboard::PutBoolean("Climb Top Limit: ", m_climbTop.Get());
             frc::SmartDashboard::PutBoolean("Climb Bottom Limit: ", m_climbBottom.Get());
        }, {this})
        .Until([this] () -> bool {
            return (m_climbBottom.Get() || m_climbTop.Get());
        });

    // return frc2::ConditionalCommand(
    //                                 frc2::CommandPtr([this] {m_climbMotor.Set(-1);}),
    //                                 frc2::CommandPtr([this] {m_climbMotor.Set(0);}),
    //                                 [this] () -> bool {return (m_climbBottom.Get() && m_climbTop.Get());});
                        
}


//frc2::CommandPtr Climb::StopClimb() {
//    return Run([this] {m_climbMotor.Set(0);});
//}

