#include "subsystems/Intake.h"
//Please look at intake.h for documentation



#include <frc/simulation/FlywheelSim.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/simulation/SimDeviceSim.h>

class IntakeSimulation
{
public:
  IntakeSimulation(Intake &intake):
    m_intakeMotorSim{"SPARK MAX ", IntakeConstants::kIntakeMotorPort},
    m_armMotorSim{intake.m_arm.GetSimCollection()},
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

  // models the physics of the components
  frc::sim::FlywheelSim m_intakeModel;
  frc::sim::SingleJointedArmSim m_armModel;
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
  m_arm.SetInverted(true);
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
  m_sim_state->m_intakeMotorSim.GetDouble("Current").Set(
    m_sim_state->m_intakeModel.GetCurrentDraw().value()
  );

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
