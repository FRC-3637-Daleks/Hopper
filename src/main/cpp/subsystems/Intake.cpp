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
    m_limitSwitchSim{intake.m_limitSwitchIntake},
    m_breakBeamSim{intake.m_breakbeam},
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
  frc::sim::DIOSim m_limitSwitchSim;

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
  m_arm.Config_kP(IntakeConstants::kPIDLoopIdx, IntakeConstants::kF, IntakeConstants::kTimeoutMs);
  m_arm.Config_kI(IntakeConstants::kPIDLoopIdx, IntakeConstants::kF, IntakeConstants::kTimeoutMs);
  m_arm.Config_kD(IntakeConstants::kPIDLoopIdx, IntakeConstants::kF, IntakeConstants::kTimeoutMs);
   
}

void Intake::Periodic() {
  frc::SmartDashboard::PutNumber("Intake/Goal", goal);
  frc::SmartDashboard::PutNumber("Intake/Intake Motor Percent Out", m_intake.Get());
  frc::SmartDashboard::PutNumber("Intake/Arm Motor Percent Out", m_arm.Get());
  frc::SmartDashboard::PutNumber("Intake/Arm Motor Position", m_arm.GetSelectedSensorPosition());
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

frc2::CommandPtr Intake::IntakeIn() {
  return frc2::cmd::RunEnd([this] {
    VertIntake();
    OnIntake();
  },
  [this] { OffIntake(); });
}

frc2::CommandPtr Intake::IntakeOut() {
  return frc2::cmd::RunEnd([this] {
    InvertIntake();
    OnIntake();
  },
  [this] { OffIntake(); });
}

Intake::~Intake() {}

/**
 * I could proboly consolidate OutputShooterIntake and OutputAMPIntake into OnIntake
 * by passing peramiters to it but I will do that later
 * (ie passing voltage and witch way to spin)
 * 
 * May also be possible to consolidate IsAtAMP and IsAtSpeaker by using goal 
 * variable and having a function that sees if GetArmDiffrence and current 
 * position has a diffrence of less or equal to kAllowableMarginOfError
*/
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


/**************************SIMULATION***************************/
void Intake::SimulationPeriodic()
{
  using namespace IntakeConstants;
  if (!m_sim_state) return;

  // Simulate intake motor
  units::volt_t applied_voltage{
    m_sim_state->m_intakeMotorSim.GetDouble("Applied Output").Get()};

  m_sim_state->m_intakeModel.SetInputVoltage(applied_voltage);
  m_sim_state->m_intakeModel.Update(20_ms);
  m_sim_state->m_intakeMotorSim.GetDouble("Velocity").Set(
    m_sim_state->m_intakeModel.GetAngularVelocity()
      .convert<units::revolutions_per_minute>().value()
  );
  m_sim_state->m_intakeMotorSim.GetDouble("Motor Current").Set(
    m_sim_state->m_intakeModel.GetCurrentDraw().value()
  );

  // Simulate game piece
  if (m_sim_state->m_notePosition >= 0_in) {
    const auto note_velocity = m_sim_state->m_intakeModel.GetAngularVelocity()*kWheelCircum;
    m_sim_state->m_notePosition += note_velocity*20_ms;
    if (m_sim_state->m_notePosition > kIntakeLength)
    {
      m_sim_state->m_notePosition = -1.0_in; // dropped the piece
    }
    else if (m_sim_state->m_notePosition < kIntakeSensorPosition)
    {
      m_sim_state->m_breakBeamSim.SetValue(kBeamBroken);
      if (m_sim_state->m_notePosition < 0_in)
      {
        m_sim_state->m_notePosition = 0_in;  // piece cant come further in
      }
    }
  } else {
    m_sim_state->m_breakBeamSim.SetValue(kBeamClear);
  }
  frc::SmartDashboard::PutNumber("Intake/sim/note_position_inches",
    m_sim_state->m_notePosition.value());

  // Simulate arm
  m_sim_state->m_armMotorSim.SetBusVoltage(
    frc::RobotController::GetBatteryVoltage().value());
  
  m_sim_state->m_armModel.SetInputVoltage(
    units::volt_t{m_sim_state->m_armMotorSim.GetMotorOutputLeadVoltage()}
  );
  m_sim_state->m_armModel.Update(20_ms);

  const units::degree_t arm_angle{m_sim_state->m_armModel.GetAngle()};
  const auto sensor_angle_reading = 
    kArmSensorFullExtend + kAngleToSensor*(arm_angle - kMinAngle);
  m_sim_state->m_armMotorSim.SetAnalogPosition(sensor_angle_reading);

  // Talon expects speed in terms of "sensor units per 100ms"
  // hence multiplying by 100_ms
  const units::degrees_per_second_t arm_speed{m_sim_state->m_armModel.GetVelocity()};
  const auto sensor_speed_reading = arm_speed*kAngleToSensor*100_ms;
  m_sim_state->m_armMotorSim.SetAnalogVelocity(sensor_speed_reading);
  m_sim_state->m_armMotorSim.SetLimitFwd(
    m_sim_state->m_armModel.HasHitUpperLimit()
  );
  m_sim_state->m_armMotorSim.SetLimitRev(
    m_sim_state->m_armModel.HasHitLowerLimit()
  );
  m_sim_state->m_armMotorSim.SetSupplyCurrent(
    m_sim_state->m_armModel.GetCurrentDraw().value()
  );
}

void Intake::SimulateNotePickup()
{
  /* Piece grabs off floor if arm is down,
   * intake is spinning inward, and there isn't already a note inside
   */
  if (m_sim_state->m_notePosition < 0_in 
    && m_sim_state->m_intakeModel.GetAngularVelocity() < 0_rad_per_s
    && m_sim_state->m_armModel.GetAngle() < IntakeConstants::kMinAngle + 5_deg)
  {
    m_sim_state->m_notePosition = 13_in;
  }
}