#include "subsystems/Intake.h"

Intake::Intake() {
  // Implementation of subsystem constructor goes here.
}

frc2::CommandPtr Intake::IntakeCommand() {
  //Please look at header file for more design details

  return StartEnd( 
    [this] {
        //#1 hasent broken   & not broken = nothing has entered yet                    keep going
        //#2 hasent broken   & is broken  = first frame intake                         keep going
        //#3 has been broken & is broken  = is intaking                                keep going
        //#4 has been broken & not broken = ring has gone in and past the sensor,      stop

        //If false (has not been broken before)
        if (!hasBeenBroken) {
            hasBeenBroken = !IntakeBreakBeam.Get(); //when break beam broken (false), has been brokem is true
        }

        frc2::ConditionalCommand( 
            frc2::RunCommand([this] {intake.Set(0.0); }), //stop (#4)
            frc2::RunCommand([this] {intake.Set(1.0); }), //keep going (is one of the following: #1, #2, #3)
            [this] () -> bool { return hasBeenBroken /*has been broken*/ && IntakeBreakBeam.Get();/*not broken*/ })/*.ToPtr()*/;}, 
  
    [this] {intake.Set(0); 
            hasBeenBroken=false;}
  );

}

//Moves Arm to specific encoder position for intaking notes
frc2::CommandPtr Intake::IntakeArmIntake(int targetPos) {
  
}

//Moves Arm to specific encoder position for AMP
frc2::CommandPtr Intake::IntakeArmAMP(int targetPos) {

}

//Moves Arn to specific endoer position for Speaker
frc2::CommandPtr Intake::IntakeArmSpeaker(int targetPos) {

}

void Intake::IntakeOn() {
    frc::SmartDashboard::PutNumber("Intake Moter output", intake.Get());
    intake.Set(1);
}
void Intake::IntakeOff() {
    frc::SmartDashboard::PutNumber("Intake Moter output", intake.Get());
    intake.Set(0);
}
void Intake::IntakeRotateUp() {
    frc::SmartDashboard::PutNumber("Intake Rotate Moter output", rotate.Get());
    rotate.Set(1);
}
void Intake::IntakeRotateDown() {
    frc::SmartDashboard::PutNumber("Intake Rotate Moter output", rotate.Get());
    rotate.Set(-1);
}
void Intake::IntakeRotateStop() {
    frc::SmartDashboard::PutNumber("Intake Rotate Moter output", rotate.Get());
    rotate.Set(0);
}