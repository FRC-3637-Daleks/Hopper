#include "subsystems/Intake.h"
//Please look at intake.h for documentation

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/simulation/FlywheelSim.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc/simulation/DIOSim.h>

class IntakeSimulation
{
public:
  IntakeSimulation(Intake &intake):
    m_intakeMotorSim{"SPARK MAX ", IntakeConstants::kIntakeMotorPort},
    m_armMotorSim{intake.m_arm.GetSimCollection()},
    m_breakBeamSim{intake.m_breakBeamFront},
    m_intakeModel{frc::DCMotor::NeoVortex(1), 2, IntakeConstants::kWheelMoment},
    m_armModel{
      IntakeConstants::kWindowMotor, IntakeConstants::kArmGearing,
      IntakeConstants::kArmMoment, IntakeConstants::kArmRadius,
      IntakeConstants::kMinAngle, IntakeConstants::kMaxAngle,
      !IntakeConstants::kGravityCompensation, IntakeConstants::kMaxAngle
    }
  {}

public:
  frc::sim::SimDeviceSim m_intakeMotorSim;
  ctre::phoenix::motorcontrol::TalonSRXSimCollection &m_armMotorSim;
  frc::sim::DIOSim m_breakBeamSim;

  // models the physics of the components
  frc::sim::FlywheelSim m_intakeModel;
  frc::sim::SingleJointedArmSim m_armModel;

  // negative for no note, 13in for note at edge, 0 for note fully in
  units::inch_t m_notePosition{-1};
};

Intake::Intake():
  m_sim_state(new IntakeSimulation(*this)) {
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
  m_arm.Config_kP(IntakeConstants::kPIDLoopIdx, IntakeConstants::kP, IntakeConstants::kTimeoutMs);
  m_arm.Config_kI(IntakeConstants::kPIDLoopIdx, IntakeConstants::kI, IntakeConstants::kTimeoutMs);
  m_arm.Config_kD(IntakeConstants::kPIDLoopIdx, IntakeConstants::kD, IntakeConstants::kTimeoutMs);

  /**
   * I am just pasting from documentation for this motion magic config and stuff
  */

  // set Motion Magic settings
  m_arm.ConfigMotionCruiseVelocity(20); // 80 rps = 16384 ticks/100ms cruise velocity
  m_arm.ConfigMotionAcceleration(20); // 160 rps/s = 32768 ticks/100ms/s acceleration
  m_arm.ConfigMotionSCurveStrength(0); // s-curve smoothing strength of 3

  // periodic, run Motion Magic with slot 0 configs
  m_arm.SelectProfileSlot(0, 0);
   
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



/*
_____
*****
-|    |
*****
-----
*/

frc2::CommandPtr Intake::AutoIntake() {
  return Run([this] {OnIntake();}).Until([this] () -> bool {return (GetStateBreakBeamBackIntake()  || GetStateLimitSwitchIntake());});
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
  m_goal = IntakeConstants::IntakeArmAMPPos;
  frc::SmartDashboard::PutNumber("Goal: ", m_goal);
  m_arm.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic/*Position*/, IntakeConstants::IntakeArmAMPPos);
}

void Intake::IntakeArmSpeaker() {
  m_goal = IntakeConstants::IntakeArmSpeakerPos;
  frc::SmartDashboard::PutNumber("Goal: ", m_goal);
  m_arm.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic/*Position*/, IntakeConstants::IntakeArmSpeakerPos);
}

void Intake::IntakeArmIntake() {
  m_goal = IntakeConstants::IntakeArmIntakePos;
  frc::SmartDashboard::PutNumber("Goal: ", m_goal);
  m_arm.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic/*Position*/, IntakeConstants::IntakeArmIntakePos);
}

frc2::CommandPtr Intake::IntakeArmAMPCommand() {
  return Run([this] {IntakeArmAMP();})
  .Until([this] () -> bool {return GetArmDiffrence() < IntakeConstants::kAllowableMarginOfError;});
}

frc2::CommandPtr Intake::IntakeArmSpeakerCommand() {
  return Run([this] {IntakeArmSpeaker();})
  .Until([this] () -> bool {return GetArmDiffrence() < IntakeConstants::kAllowableMarginOfError;});
}

frc2::CommandPtr Intake::IntakeArmIntakeCommand() {
  return Run([this] {IntakeArmIntake();})
  .Until([this] () -> bool {return GetArmDiffrence() < IntakeConstants::kAllowableMarginOfError;});
}

bool Intake::GetStateLimitSwitchIntake() {
  return m_limitSwitchIntake.Get();
}

bool Intake::GetStateBreakBeamFrontIntake() {
  return m_breakBeamFront.Get();
}

bool Intake::GetStateBreakBeamBackIntake() {
  return m_breakBeamBack.Get();
}

bool Intake::GetStateShooterBeamIntake() {
  return m_shooterBreakBeam.Get();
}

int Intake::GetArmDiffrence() {
  return m_goal - m_arm.GetSelectedSensorPosition();
}

bool Intake::IsAtAMP() {
  return abs(m_arm.GetSelectedSensorPosition() == IntakeConstants::IntakeArmAMPPos) <= IntakeConstants::kAllowableMarginOfError;
}

bool Intake::IsAtSpeaker() {
  return abs(m_arm.GetSelectedSensorPosition() == IntakeConstants::IntakeArmSpeakerPos) <= IntakeConstants::kAllowableMarginOfError;
}