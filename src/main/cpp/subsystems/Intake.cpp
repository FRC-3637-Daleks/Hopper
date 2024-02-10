#include "subsystems/Intake.h"
//Please look at intake.h for documentation

Intake::Intake() {
  //https://github.com/CrossTheRoadElec/Phoenix5-Examples/blob/master/C%2B%2B%20General/PositionClosedLoop/src/main/cpp/Robot.cpp

  m_arm.ConfigFactoryDefault();

  m_arm.SetSelectedSensorPosition(0 /*starting position*/, IntakeConstants::kPIDLoopIdx, IntakeConstants::kTimeoutMs);

  m_arm.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::Analog/*idk if correct*/, IntakeConstants::kPIDLoopIdx, IntakeConstants::kTimeoutMs);
  //I think this sets the direction of the sensor, not too sure
  m_arm.SetSensorPhase(true/*idk if correct*/);

  //something with power or something
  m_arm.ConfigNominalOutputForward(0.0, IntakeConstants::kTimeoutMs);
  m_arm.ConfigNominalOutputReverse(0.0, IntakeConstants::kTimeoutMs);
  m_arm.ConfigPeakOutputForward(1.0, IntakeConstants::kTimeoutMs);
  m_arm.ConfigPeakOutputReverse(-1.0, IntakeConstants::kTimeoutMs);

  m_arm.Config_kF(IntakeConstants::kPIDLoopIdx, IntakeConstants::kF, IntakeConstants::kTimeoutMs);
  m_arm.Config_kP(IntakeConstants::kPIDLoopIdx, IntakeConstants::kF, IntakeConstants::kTimeoutMs);
  m_arm.Config_kI(IntakeConstants::kPIDLoopIdx, IntakeConstants::kF, IntakeConstants::kTimeoutMs);
  m_arm.Config_kD(IntakeConstants::kPIDLoopIdx, IntakeConstants::kF, IntakeConstants::kTimeoutMs);
   
}


frc2::CommandPtr Intake::IntakeRing() {
  return this->RunOnce([this] {
    IntakeArmIntake();})
    .AndThen(this->StartEnd(
      [this] {frc2::ConditionalCommand(
                                      frc2::RunCommand([this] {OnIntake();}),
                                      frc2::RunCommand([this] {OffIntake();}),
                                      [this] () -> bool {return GetStateBreakBeamIntake()/*when unbroken*/ || !GetStateLimitSwitchIntake()/*when untouched*/;});
      },
      [this] {OffIntake();}
    ));
}

frc2::CommandPtr Intake::ShootOnAMP() {
  return this->RunOnce([this] {
    IntakeArmAMP();})
    .AndThen(StartEnd([this] {OutputAMPIntake();},
                      [this] {OffIntake();}));
}

frc2::CommandPtr Intake::OutputToShooter() {
  return this->RunOnce([this] {
    IntakeArmSpeaker();})
    .AndThen(this->RunEnd(
      [this] {frc2::ConditionalCommand(
                                      frc2::RunCommand([this] {OutputShooterIntake();}),
                                      frc2::RunCommand([this] {OffIntake();}),
                                      [this] () -> bool {return true/*replace with break beam from the shooter*/;});
      },
      [this] {OffIntake();}
    ));
}



void Intake::InvertIntake() {
  m_intake.SetInverted(true);
}

void Intake::VertIntake() {
  m_arm.SetInverted(false);
}

void Intake::OnIntake() {
  VertIntake();
  m_intake.SetVoltage(IntakeConstants::kIntakeVoltage);
}

void Intake::OffIntake() {
  m_intake.SetVoltage(IntakeConstants::kOffVoltage);
}

void Intake::OutputShooterIntake() {
  InvertIntake();
  m_intake.SetVoltage(IntakeConstants::kShooterVoltage);
}

void Intake::OutputAMPIntake() {
  InvertIntake();
  m_intake.SetVoltage(IntakeConstants::kAMPVoltage);
}

void Intake::IntakeArmAMP() {
  goal = IntakeConstants::IntakeArmAMPPos;
  m_arm.Set(ctre::phoenix::motorcontrol::ControlMode::Position, IntakeConstants::IntakeArmAMPPos);
}

void Intake::IntakeArmSpeaker() {
  goal = IntakeConstants::IntakeArmSpeakerPos;
  m_arm.Set(ctre::phoenix::motorcontrol::ControlMode::Position, IntakeConstants::IntakeArmSpeakerPos);
}

void Intake::IntakeArmIntake() {
  goal = IntakeConstants::IntakeArmIntakePos;
  m_arm.Set(ctre::phoenix::motorcontrol::ControlMode::Position, IntakeConstants::IntakeArmIntakePos);
}

bool Intake::GetStateLimitSwitchIntake() {
  return m_limitSwitchIntake.Get();
}

bool Intake::GetStateBreakBeamIntake() {
  return m_breakbeam.Get();
}

bool Intake::GetStateShooterBeamIntake() {
  return m_shooterBreakBeam.Get();
}

int Intake::GetArmDiffrence() {
  return goal - m_arm.GetSelectedSensorPosition();
}

bool Intake::IsAtAMP() {
  return abs(m_arm.GetSelectedSensorPosition() == IntakeConstants::IntakeArmAMPPos) <= IntakeConstants::kAllowableMarginOfError;
}

bool Intake::IsAtSpeaker() {
  return abs(m_arm.GetSelectedSensorPosition() == IntakeConstants::IntakeArmSpeakerPos) <= IntakeConstants::kAllowableMarginOfError;
}