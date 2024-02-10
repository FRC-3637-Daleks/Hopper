//Breakbeams + possibly absolute encoders

// ClimbSubsystem.cpp
// object name = digitalInput
#include "subsystems/Climb.h"
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

frc2::CommandPtr Climb::ClimbCommand(double input) {
    frc::SmartDashboard::PutNumber("Climb Input: ", input);
    frc::SmartDashboard::PutBoolean("Climb Top Limit: ", m_climbTop.Get());
    frc::SmartDashboard::PutBoolean("Climb Bottom Limit: ", m_climbBottom.Get());
    return frc2::cmd::RunEnd([this, input] {
            m_climbMotor.Set(input);
        }, [this] () {
             m_climbMotor.Set(0);
             frc::SmartDashboard::PutBoolean("Climb Top Limit: ", m_climbTop.Get());
             frc::SmartDashboard::PutBoolean("Climb Bottom Limit: ", m_climbBottom.Get());
        }, {this})
        .Until([this] () -> bool {
            return (m_climbBottom.Get() || m_climbTop.Get());
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

