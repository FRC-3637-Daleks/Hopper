#include "subsystems/Intake.h"
//Please look at intake.h for documentation

Intake::Intake() {
  rotate.RestoreFactoryDefaults(); //resets PID settings on moter

  //Applies configuration for Intake 
  m_pidController.SetP(kP);
  m_pidController.SetI(kI);
  m_pidController.SetD(kD);
  m_pidController.SetIZone(kIz);
  m_pidController.SetFF(kFF);
  m_pidController.SetOutputRange(kMinOutput, kMaxOutput);
  
}

frc2::CommandPtr Intake::IntakeCommand() {

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
            frc2::RunCommand([this] {IntakeOff();}), //stop (#4)
            frc2::RunCommand([this] {IntakeOn(); }), //keep going (is one of the following: #1, #2, #3)
            [this] () -> bool { return hasBeenBroken /*has been broken*/ && IntakeBreakBeam.Get();/*not broken now*/ })/*.ToPtr()*/;}, 
  
    [this] {IntakeOff(); 
            hasBeenBroken=false;}
  );

}


/** Set refrence documentation
 * value: In this case, for Position, measured in rotations 
   (whatever that means there measured in 100s/1000s for sparkMax)
 * ctrl: eather volts, position, velocity, current 
 * (2 more optional slots (or at least it does not throw error))
*/
//Moves arm to positition specified by IntakeArmIntakePos
frc2::CommandPtr Intake::IntakeArmIntake() {
    return RunOnce(
        [this] {m_pidController.SetReference(IntakeConstants::IntakeArmIntakePos, rev::CANSparkMax::ControlType::kPosition); });
}

//Moves arm to positition specified by IntakeArmAMPPos
frc2::CommandPtr Intake::IntakeArmAMP() {
    return RunOnce(
        [this] {m_pidController.SetReference(IntakeConstants::IntakeArmAMPPos, rev::CANSparkMax::ControlType::kPosition); });
}

//Moves arm to positition specified by IntakeArmSpeakerPos
frc2::CommandPtr Intake::IntakeArmSpeaker() {
    return RunOnce(
        [this] {m_pidController.SetReference(IntakeConstants::IntakeArmSpeakerPos, rev::CANSparkMax::ControlType::kPosition); });
}

//Outputs ring for AMP
frc2::CommandPtr Intake::IntakeAMPOutput() {
    return RunOnce(
        [this] {intake.Set(-1.0);});
}

//Outputs ring for speaker
frc2::CommandPtr Intake::IntakeSpeakerOutput() {
    return RunOnce(
        [this] {intake.Set(-0.5);});
}

//Turns on the intake
frc2::CommandPtr Intake::IntakeOn() {
    return RunOnce(
        [this] {intake.Set(1.0);});
}

//Turns off the intake
frc2::CommandPtr Intake::IntakeOff() {
    return RunOnce(
        [this] {intake.Set(0.0);});
}

