#include "subsystems/Intake.h"
// Please look at intake.h for documentation

#include <frc/DataLogManager.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

#include <frc/simulation/DIOSim.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc/simulation/SingleJointedArmSim.h>

class IntakeSimulation {
public:
  IntakeSimulation(Intake &intake)
      : m_intakeMotorSim{"SPARK MAX ", IntakeConstants::kIntakeMotorPort},
        m_armMotorSim{intake.m_arm.GetSimCollection()},
        m_breakBeamSim{intake.m_breakbeam},
        m_intakeModel{frc::DCMotor::NeoVortex(1), 1,
                      IntakeConstants::kWheelMoment},
        m_armModel{frc::DCMotor::Vex775Pro(1),
                   IntakeConstants::kArmGearing,
                   IntakeConstants::kArmMoment,
                   IntakeConstants::kArmRadius,
                   IntakeConstants::kMinAngle,
                   IntakeConstants::kMaxAngle,
                   !IntakeConstants::kGravityCompensation,
                   IntakeConstants::kMaxAngle} {}

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

Intake::Intake() : m_sim_state(new IntakeSimulation(*this)) {
  // https://github.com/CrossTheRoadElec/Phoenix5-Examples/blob/master/C%2B%2B%20General/PositionClosedLoop/src/main/cpp/Robot.cpp

  m_arm.ConfigFactoryDefault();

  m_arm.SetSelectedSensorPosition(0 /*starting position*/,
                                  IntakeConstants::kPIDLoopIdx,
                                  IntakeConstants::kTimeoutMs);
  m_arm.ConfigSelectedFeedbackSensor(
      ctre::phoenix::motorcontrol::FeedbackDevice::Analog /*idk if correct*/,
      IntakeConstants::kPIDLoopIdx, IntakeConstants::kTimeoutMs);
  // I think this sets the direction of the sensor, not too sure
  m_arm.SetSensorPhase(false /*idk if correct*/);
  m_arm.SetInverted(false);

  // something with power or something
  m_arm.ConfigNominalOutputForward(0.0, IntakeConstants::kTimeoutMs);
  m_arm.ConfigNominalOutputReverse(0.0, IntakeConstants::kTimeoutMs);
  m_arm.ConfigPeakOutputForward(1.0, IntakeConstants::kTimeoutMs);
  m_arm.ConfigPeakOutputReverse(-1.0, IntakeConstants::kTimeoutMs);

  m_arm.Config_kF(IntakeConstants::kPIDLoopIdx, IntakeConstants::kF,
                  IntakeConstants::kTimeoutMs);
  m_arm.Config_kP(IntakeConstants::kPIDLoopIdx, IntakeConstants::kP,
                  IntakeConstants::kTimeoutMs);
  m_arm.Config_kI(IntakeConstants::kPIDLoopIdx, IntakeConstants::kI,
                  IntakeConstants::kTimeoutMs);
  m_arm.Config_kD(IntakeConstants::kPIDLoopIdx, IntakeConstants::kD,
                  IntakeConstants::kTimeoutMs);

  m_arm.ConfigFeedbackNotContinuous(true);

  m_arm.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
  /**
   * I am just pasting from documentation for this motion magic config and stuff
   */

  // set Motion Magic settings
  m_arm.ConfigMotionCruiseVelocity(
      130); // 80 rps = 16384 ticks/100ms cruise velocity
  m_arm.ConfigMotionAcceleration(
      1300); // 160 rps/s = 32768 ticks/100ms/s acceleration
  m_arm.ConfigMotionSCurveStrength(0); // s-curve smoothing strength of 3

  // periodic, run Motion Magic with slot 0 configs
  m_arm.SelectProfileSlot(0, 0);

  m_arm.ConfigSupplyCurrentLimit({true, IntakeConstants::kMaxCurrent.value(),
                                  IntakeConstants::kMaxCurrent.value(), 0.1});

  frc::DataLogManager::Log(
      fmt::format("Finished initializing intake subsystem."));
}

void Intake::Periodic() {
  frc::SmartDashboard::PutNumber("Intake/Goal", m_goal);
  frc::SmartDashboard::PutNumber("Intake/Intake Motor Percent Out",
                                 m_intake.Get());
  frc::SmartDashboard::PutNumber("Intake/Arm Motor Percent Out", m_arm.Get());
  frc::SmartDashboard::PutNumber("Intake/Arm Motor Position",
                                 m_arm.GetSelectedSensorPosition());
  frc::SmartDashboard::PutBoolean("Intake/Break Beam State (raw)",
                                  m_breakbeam.Get());
  frc::SmartDashboard::PutNumber("Intake/Break Beam Broken (function)",
                                 IsIntakeBreakBeamBroken());
  frc::SmartDashboard::PutNumber("Intake/Arm Position",
                                 m_arm.GetSelectedSensorPosition());
  frc::SmartDashboard::PutBoolean("Intake/Is Intaking?", IsIntaking());
  UpdateVisualization();
}

void Intake::Emergency(double input) { m_arm.Set(input); }

frc2::CommandPtr Intake::IntakeRing() {
  return frc2::cmd::Either(IntakeArmIntakeCommand(false) // true
                               .AndThen(AutoIntake()),
                           frc2::cmd::None(), // false
                           [this]() {
                             return !IsIntakeBreakBeamBroken();
                           } // When broken = false
                           )
      .AndThen(
          IntakeArmSpeakerCommand()); // if true does not allow for inturupts
}

frc2::CommandPtr Intake::IntakeFromPlayerStation() {
  return frc2::cmd::Either(IntakeArmSourceCommand(false) // true
                               .AndThen(AutoIntake()),
                           frc2::cmd::None(), // false
                           [this]() {
                             return !IsIntakeBreakBeamBroken();
                           } // When broken = false
                           )
      .AndThen(IntakeArmSpeakerCommand());
  ;
}

frc2::CommandPtr Intake::ShootOnAMPCatch() {
  return frc2::cmd::Either(
      IntakeArmSpeakerCommand(true) // when ring
          .AndThen(Run([this] { ShootOnAMPVoid(); }).WithTimeout(1.2_s))
          .Until([this]() -> bool { // when we reach the AMP pos
            return ((m_arm.GetSelectedSensorPosition() >=
                         IntakeConstants::IntakeArmAMPPos - 10 &&
                     m_arm.GetSelectedSensorPosition() <=
                         IntakeConstants::IntakeArmAMPPos +
                             30) && // when 10 before or 30 after AMP
                    m_goal == IntakeConstants::IntakeArmAMPVelocityPos);
          }) // And Goal is set to Velocity (only happen when AMP shot)
          .AndThen(IntakeArmAMPCommand(false))
          .AndThen(Run([this] {
                     IntakeBackward();
                   }).WithTimeout(0.6_s)) // Just so it runs for a little longer
          .AndThen(IntakeOff())
          .AndThen(IntakeOff().AndThen(IntakeArmSpeakerCommand(
              true))) /*.AndThen(IntakeArmSpeakerCommand(true))*/,
      frc2::cmd::None(),                             // no ring
      [this]() { return IsIntakeBreakBeamBroken(); } // When broken = true
  );
}

frc2::CommandPtr Intake::ShootOnAMPRetract() {
  return frc2::cmd::Either(
      IntakeArmSpeakerCommand(true) // when ring
          .AndThen(Run([this] { ShootOnAMPVoid(); }).WithTimeout(1.2_s))
          .Until([this]() -> bool { // when we reach the AMP pos
            return ((m_arm.GetSelectedSensorPosition() >=
                         IntakeConstants::IntakeArmAMPPos - 10 &&
                     m_arm.GetSelectedSensorPosition() <=
                         IntakeConstants::IntakeArmAMPPos +
                             30) && // when 10 before or 30 after AMP
                    m_goal == IntakeConstants::IntakeArmAMPVelocityPos);
          }) // And Goal is set to Velocity (only happen when AMP shot)
          .AndThen(IntakeArmAMPCommand(false))
          .AndThen(Run([this] {
                     IntakeBackward();
                   }).WithTimeout(0.6_s)) // Just so it runs for a little longer
          .AndThen(IntakeOff())
          .AndThen(AutoIntake())
          .WithTimeout(2_s)
          .AndThen(IntakeOff()) /*.AndThen(IntakeArmSpeakerCommand(true))*/,
      frc2::cmd::None(),                             // no ring
      [this]() { return IsIntakeBreakBeamBroken(); } // When broken = true
  );
}

frc2::CommandPtr Intake::OutputToShooter() {
  return frc2::cmd::Either(
      IntakeArmSpeakerCommand(true)
          .Until([this]() -> bool {
            return m_arm.GetSelectedSensorPosition() <
                   IntakeConstants::IntakeArmSpeakerPos;
          }) // when ring
          .AndThen(IntakeOutSpeaker())
          .WithTimeout(0.5_s),
      frc2::cmd::None(),                             // no ring
      [this]() { return IsIntakeBreakBeamBroken(); } // When broken = true
  );
}

frc2::CommandPtr Intake::IntakeIn() {
  return RunEnd([this] { IntakeForward(); }, [this] { OffIntake(); });
}
frc2::CommandPtr Intake::IntakeOff() {
  return RunOnce([this] { OffIntake(); });
}

frc2::CommandPtr Intake::IntakeOut() {
  return RunEnd([this] { IntakeBackward(); }, [this] { OffIntake(); });
}

frc2::CommandPtr Intake::IntakeOutSpeaker() {
  return frc2::cmd::RunEnd([this] { IntakeBackwardSpeaker(); },
                           [this] { OffIntake(); });
}

frc2::CommandPtr Intake::TimedRelease() {
  return Run([this] { OffIntake(); })
      .Until([this]() -> bool {
        return (m_arm.GetSelectedSensorPosition()) >=
               IntakeConstants::IntakeArmLetGoPos;
      })
      .AndThen(IntakeOut());
}

void Intake::ShootOnAMPVoid() {
  IntakeArmAMPVelocity();
  if ((m_arm.GetSelectedSensorPosition()) >=
      IntakeConstants::IntakeArmLetGoPos) {
    IntakeBackward();
  }
}

Intake::~Intake() {}

void Intake::InitVisualization(frc::Mechanism2d *mech) {
  // position of pivot approximated in meters
  m_mech_root = mech->GetRoot("intake", 0.73152, 0.1524);

  m_mech_arm_goal = m_mech_root->Append<frc::MechanismLigament2d>(
      "arm goal",
      IntakeConstants::kArmRadius.convert<units::meter>()
          .value(),               // line length in meters
      0_deg,                      // line angle
      6,                          // line width in pixels
      frc::Color8Bit{20, 200, 20} // RGB, green
  );

  m_mech_arm_mm_setpoint = m_mech_root->Append<frc::MechanismLigament2d>(
      "arm motion magic",
      IntakeConstants::kArmRadius.convert<units::meter>().value(), 0_deg, 2,
      frc::Color8Bit{80, 80, 200} // blueish
  );

  m_mech_arm = m_mech_root->Append<frc::MechanismLigament2d>(
      "arm", IntakeConstants::kArmRadius.convert<units::meter>().value(), 0_deg,
      6, frc::Color8Bit{200, 20, 200} // magenta
  );

  m_mech_spinner = m_mech_arm->Append<frc::MechanismLigament2d>(
      "wheel",
      (IntakeConstants::kWheelDiameter / 2).convert<units::meter>().value(),
      0_deg, 2, frc::Color8Bit{20, 200, 200} // cyan
  );

  m_mech_note = m_mech_root->Append<frc::MechanismLigament2d>(
      "note",
      0.24384,                               // lets say 0.8_ft with compression
      0_deg, 1, frc::Color8Bit{240, 20, 180} // orange
  );
}

void Intake::UpdateVisualization() {
  // Visualization wasn't initialized
  if (!m_mech_root)
    return;

  PreviousSensorPosition = m_arm.GetSelectedSensorPosition();

  m_mech_arm->SetAngle(
      IntakeConstants::sensorToAngle(m_arm.GetSelectedSensorPosition()));
  m_mech_arm_goal->SetAngle(IntakeConstants::sensorToAngle(m_goal));
  m_mech_arm_mm_setpoint->SetAngle(
      IntakeConstants::sensorToAngle(m_arm.GetActiveTrajectoryPosition()));
  m_mech_spinner->SetAngle(units::degree_t{m_mech_spinner->GetAngle()} +
                           m_intake.GetAppliedOutput() * 66.66_deg);
  m_mech_note->SetAngle(units::degree_t{m_mech_arm->GetAngle()});
  if (IsIntakeBreakBeamBroken()) {
    m_mech_note->SetLineWeight(10);
    m_mech_note->SetColor({200, 200, 40});
  } else {
    m_mech_note->SetLineWeight(1);
    m_mech_note->SetColor({200, 20, 200});
  }
}

/**
 * I could proboly consolidate OutputShooterIntake and OutputAMPIntake into
 * OnIntake by passing peramiters to it but I will do that later (ie passing
 * voltage and witch way to spin)
 *
 * May also be possible to consolidate IsAtAMP and IsAtSpeaker by using goal
 * variable and having a function that sees if GetArmDiffrence and current
 * position has a diffrence of less or equal to kAllowableMarginOfError
 */
/*
_____
*****
-|    |
*****
-----
*/

frc2::CommandPtr Intake::AutoIntake() {
  return Run([this] { IntakeForward(); })
      .Until([this]() -> bool { return (IsIntakeBreakBeamBroken()); })
      .AndThen([this] { OffIntake(); })
      .HandleInterrupt([this] { OffIntake(); });
}

void Intake::IntakeForward() { // in
  m_intake.SetVoltage(3_V);
}

void Intake::IntakeBackward() { // out, (i was adjusting the voltage for amp)
  m_intake.SetVoltage(-1 * (1.75_V));
}

void Intake::IntakeBackwardSpeaker() { m_intake.SetVoltage(-1 * (1.75_V)); }

void Intake::OffIntake() { m_intake.SetVoltage(0_V); }

void Intake::IntakeArmAMP() {
  m_goal = IntakeConstants::IntakeArmAMPPos;
  m_arm.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic /*Position*/,
            IntakeConstants::IntakeArmAMPPos);
  // frc::ArmFeedforward ff{0.01V, }
}

void Intake::IntakeArmSpeaker() {
  m_goal = IntakeConstants::IntakeArmSpeakerPos;
  m_arm.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic /*Position*/,
            IntakeConstants::IntakeArmSpeakerPos);
}

void Intake::IntakeArmIntake() {
  m_goal = IntakeConstants::IntakeArmIntakePos;
  m_arm.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic /*Position*/,
            IntakeConstants::IntakeArmIntakePos);
}

void Intake::IntakeArmSource() {
  m_goal = IntakeConstants::IntakeArmSourceIntakePos;
  m_arm.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic /*Position*/,
            IntakeConstants::IntakeArmSourceIntakePos);
}

bool Intake::IsAtWantedPosition(int goal) {
  return (abs(goal - m_arm.GetSelectedSensorPosition()) <
          IntakeConstants::kAllowableMarginOfError);
}

bool Intake::IsIntaking() { return m_intake.Get() > 0.0; }

void Intake::IntakeArmAMPVelocity() {
  m_goal = IntakeConstants::IntakeArmAMPVelocityPos;
  m_arm.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, 70);
}

frc2::CommandPtr Intake::IntakeArmAMPCommand(bool wait) {

  if (wait) {
    return this->Run([this] { IntakeArmAMP(); }).Until([this]() -> bool {
      return IsAtWantedPosition(IntakeConstants::IntakeArmAMPPos);
    });
  }
  return RunOnce([this] { IntakeArmAMP(); });
}

frc2::CommandPtr Intake::IntakeArmSourceCommand(bool wait) {

  if (wait) {
    return this->Run([this] { IntakeArmSource(); }).Until([this]() -> bool {
      return IsAtWantedPosition(IntakeConstants::IntakeArmSourceIntakePos);
    });
  }
  return RunOnce([this] { IntakeArmSource(); });
}

frc2::CommandPtr Intake::IntakeArmSpeakerCommand(bool wait) {

  if (wait) {
    return this->Run([this] { IntakeArmSpeaker(); }).Until([this]() -> bool {
      return IsAtWantedPosition(IntakeConstants::IntakeArmSpeakerPos);
    });
  }
  return RunOnce([this] { IntakeArmSpeaker(); });
}

frc2::CommandPtr Intake::IntakeArmIntakeCommand(bool wait) {

  if (wait) {
    return this->Run([this] { IntakeArmIntake(); }).Until([this]() -> bool {
      return IsAtWantedPosition(IntakeConstants::IntakeArmIntakePos);
    });
  }
  return RunOnce([this] { IntakeArmIntake(); });
}

frc2::CommandPtr Intake::IntakeArmAMPVelocityCommand(bool wait) {

  if (wait) {
    return this->Run([this] { IntakeArmAMPVelocity(); })
        .Until([this]() -> bool {
          return IsAtWantedPosition(IntakeConstants::IntakeArmAMPPos);
        });
  }
  return RunOnce([this] { IntakeArmIntake(); });
}

frc2::CommandPtr Intake::IdleIntakeCommand() { return frc2::cmd::None(); }

bool Intake::IsIntakeBreakBeamBroken() { // when broken true
  return !(m_breakbeam.Get());
}

int Intake::GetArmDiffrence() {
  return abs(m_goal - m_arm.GetSelectedSensorPosition());
}

bool Intake::IsAtAMP() {
  return abs(m_arm.GetSelectedSensorPosition() ==
             IntakeConstants::IntakeArmAMPPos) <=
         IntakeConstants::kAllowableMarginOfError;
}

bool Intake::IsAtSpeaker() {
  return abs(m_arm.GetSelectedSensorPosition() ==
             IntakeConstants::IntakeArmSpeakerPos) <=
         IntakeConstants::kAllowableMarginOfError;
}

/**************************SIMULATION***************************/
void Intake::SimulationPeriodic() {
  using namespace IntakeConstants;
  if (!m_sim_state)
    return;

  // Simulate intake motor
  units::volt_t applied_voltage{
      -m_sim_state->m_intakeMotorSim.GetDouble("Applied Output").Get()};

  m_sim_state->m_intakeModel.SetInputVoltage(applied_voltage);
  m_sim_state->m_intakeModel.Update(20_ms);
  m_sim_state->m_intakeMotorSim.GetDouble("Velocity")
      .Set(-m_sim_state->m_intakeModel.GetAngularVelocity()
                .convert<units::revolutions_per_minute>()
                .value());
  m_sim_state->m_intakeMotorSim.GetDouble("Motor Current")
      .Set(m_sim_state->m_intakeModel.GetCurrentDraw().value());

  // Simulate game piece
  if (m_sim_state->m_notePosition >= 0_in) {
    const auto note_velocity =
        m_sim_state->m_intakeModel.GetAngularVelocity() * kWheelCircum;
    m_sim_state->m_notePosition += note_velocity * 20_ms;
    if (m_sim_state->m_notePosition > kIntakeLength) {
      m_sim_state->m_notePosition = -1.0_in; // dropped the piece
    } else if (m_sim_state->m_notePosition < kIntakeSensorPosition) {
      m_sim_state->m_breakBeamSim.SetValue(kBeamBroken);
      if (m_sim_state->m_notePosition < 0_in) {
        m_sim_state->m_notePosition = 0_in; // piece cant come further in
      }
    }
  } else {
    m_sim_state->m_breakBeamSim.SetValue(not kBeamBroken);
  }
  frc::SmartDashboard::PutNumber("Intake/sim/note_position_inches",
                                 m_sim_state->m_notePosition.value());

  // Simulate arm
  m_sim_state->m_armMotorSim.SetBusVoltage(
      frc::RobotController::GetBatteryVoltage().value());

  // Voltage is reversed on real robot
  m_sim_state->m_armModel.SetInputVoltage(
      -units::volt_t{m_sim_state->m_armMotorSim.GetMotorOutputLeadVoltage()});
  m_sim_state->m_armModel.Update(20_ms);

  const units::degree_t arm_angle{m_sim_state->m_armModel.GetAngle()};
  const auto sensor_angle_reading =
      kArmSensorFullExtend + kAngleToSensor * (arm_angle - kMinAngle);
  m_sim_state->m_armMotorSim.SetAnalogPosition(sensor_angle_reading);

  // Talon expects speed in terms of "sensor units per 100ms"
  // hence multiplying by 100_ms
  const units::degrees_per_second_t arm_speed{
      m_sim_state->m_armModel.GetVelocity()};
  const auto sensor_speed_reading = arm_speed * kAngleToSensor * 100_ms;
  m_sim_state->m_armMotorSim.SetAnalogVelocity(sensor_speed_reading);
  // Real robot has the reverse limit at the full retract position
  m_sim_state->m_armMotorSim.SetLimitRev(
      m_sim_state->m_armModel.HasHitUpperLimit());
  m_sim_state->m_armMotorSim.SetSupplyCurrent(
      m_sim_state->m_armModel.GetCurrentDraw().value());
}

bool Intake::SimulateNotePickup() {
  /* Piece grabs off floor if arm is down,
   * intake is spinning inward, and there isn't already a note inside
   */
  if (m_sim_state->m_notePosition < 0_in &&
      m_sim_state->m_intakeModel.GetAngularVelocity() < 0_rad_per_s &&
      m_sim_state->m_armModel.GetAngle() <
          IntakeConstants::kMinAngle + 20_deg) {
    m_sim_state->m_notePosition = IntakeConstants::kIntakeLength;
    return true;
  }

  return false;
}
