// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"


#include <frc2/command/Commands.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>

#include <frc/simulation/FlywheelSim.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc/simulation/DIOSim.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/DriverStation.h>

class ShooterSimulation
{
public:
  ShooterSimulation(Shooter &shooter):
    m_leadShooterSim{"SPARK Flex ", ShooterConstants::kFlywheelLeadMotorPort},
    m_followerShooterSim{"SPARK Flex ", ShooterConstants::kFlywheelFollowMotorPort},
    m_aimMotorSim{shooter.m_pivot.GetSimCollection()},
    m_shooterModel{frc::DCMotor::NeoVortex(1), 1, ShooterConstants::kWheelMoment},
    m_armModel{
      ShooterConstants::kWindowMotor, ShooterConstants::kArmGearing,
      ShooterConstants::kArmMoment, ShooterConstants::kArmRadius,
      ShooterConstants::kMinAngle, ShooterConstants::kMaxAngle,
      false, ShooterConstants::kMaxAngle
    }
  {}

public:
  frc::sim::SimDeviceSim m_leadShooterSim, m_followerShooterSim;
  ctre::phoenix::motorcontrol::TalonSRXSimCollection &m_aimMotorSim;

  // models the physics of the components
  frc::sim::FlywheelSim m_shooterModel;
  frc::sim::SingleJointedArmSim m_armModel;
};

Shooter::Shooter(): m_sim_state(new ShooterSimulation(*this)) {
// Implementation of subsystem constructor goes here.
// Needs arguments to work between power cycles!!
// Resets config perameters
  m_followMotor.RestoreFactoryDefaults();
  m_leadMotor.RestoreFactoryDefaults();
  m_pivot.ConfigFactoryDefault();

  m_pivot.SetSelectedSensorPosition(0, ShooterConstants::kPIDLoopIdx, ShooterConstants::kTimeoutMs);

  m_pivot.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::Analog, ShooterConstants::kPIDLoopIdx, ShooterConstants::kTimeoutMs);

  m_pivot.SetSensorPhase(false);
  m_pivot.SetInverted(false);

  m_pivot.ConfigNominalOutputForward(0, ShooterConstants::kTimeoutMs);
  m_pivot.ConfigNominalOutputReverse(0, ShooterConstants::kTimeoutMs);
  m_pivot.ConfigPeakOutputForward(1.0, ShooterConstants::kTimeoutMs);
  m_pivot.ConfigPeakOutputReverse(-1.0, ShooterConstants::kTimeoutMs);
  
  m_pivot.Config_kF(ShooterConstants::kPIDLoopIdx, ShooterConstants::kFPivot, ShooterConstants::kTimeoutMs);
  m_pivot.Config_kP(ShooterConstants::kPIDLoopIdx, ShooterConstants::kPPivot, ShooterConstants::kTimeoutMs);
  m_pivot.Config_kI(ShooterConstants::kPIDLoopIdx, ShooterConstants::kIPivot, ShooterConstants::kTimeoutMs);
  m_pivot.Config_kD(ShooterConstants::kPIDLoopIdx, ShooterConstants::kDPivot, ShooterConstants::kTimeoutMs);
//Motors following + leading

  m_pivot.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::MotionMagic, 0);
  
  
  // set Motion Magic settings
  m_pivot.ConfigMotionCruiseVelocity(320); // 80 rps = 16384 ticks/100ms cruise velocity
  m_pivot.ConfigMotionAcceleration(80); // 160 rps/s = 32768 ticks/100ms/s acceleration
  m_pivot.ConfigMotionSCurveStrength(0); // s-curve smoothing strength of 3

  // periodic, run Motion Magic with slot 0 configs
  m_pivot.SelectProfileSlot(0, 0);

  m_leadMotor.SetInverted(true);

  m_followMotor.Follow(m_leadMotor,true);

  m_followMotor.SetInverted(false);
  
}

Shooter::~Shooter() {}

void Shooter::InitVisualization(frc::Mechanism2d* mech)
{
  auto root = mech->GetRoot("shooter", 1.5, 1.5);

  m_mech_pivot_goal = root->Append<frc::MechanismLigament2d>(
    "aim goal",  // name
    1,  // feet long
    180_deg,  // start angle
    4,  // pixel width
    frc::Color8Bit{20, 200, 20}  // RGB, green
  );

  m_mech_pivot = root->Append<frc::MechanismLigament2d>(
    "aim",
    1,
    180_deg,
    10,
    frc::Color8Bit{240, 240, 240}  // white
  );
}

void Shooter::UpdateVisualization()
{
  const auto sensor_goal = m_pivot.GetClosedLoopTarget();
  const auto angle_goal = (sensor_goal - ShooterConstants::kMinAimSensor)/ShooterConstants::kAngleToSensor;
  m_mech_pivot_goal->SetAngle(180_deg - angle_goal);

  const auto sensor_measured = m_pivot.GetSelectedSensorPosition();
  const auto angle_measured = (sensor_measured - ShooterConstants::kMinAimSensor)/ShooterConstants::kAngleToSensor;
  m_mech_pivot->SetAngle(180_deg - angle_measured);

  // scale blueness of shooter on flywheel speed
  const auto wheel_vel = m_leadMotor.GetAppliedOutput();
  m_mech_pivot->SetColor({240 - int(wheel_vel*200), 240 - int(wheel_vel*200), 240});
}

//Runs both shooting motors
void Shooter::RunShootMotor() {

//Starts
   m_leadMotor.SetVoltage(1.0_V);
   
}

//Stops both shooting motors
void Shooter::StopShootMotor() {

//Stops
  m_leadMotor.StopMotor();

}

//Runs pivoting motor
void Shooter::RunTalonMotor() {
//Runs
m_pivot.SetVoltage(1.0_V);

}

float pow(float d, int power) {
  float temp = d;
  for (int i = 0; i < power-1; i++) {
    temp = temp * d;
  }
  return temp;
}

float Shooter::KevensCoolEquasion(float d) {
  return (1.57 - (0.175*d) - (2.14*.001*(pow(d, 2)))+(3.58*.001*pow(d,3))-(5.3*.0001*pow(d,4))+(4.16*.00001*pow(d,5))-(1.92*.000001*pow(d,6))+(4.95*.00000001*pow(d,7))-(5.52*.0000000001*pow(d,8)));
}

//Stop pivoting motor
void Shooter::StopTalonMotor() {
//Stops
 m_pivot.StopMotor();

}

void Shooter::SetPivotMotor(double encoderPosition) {
  m_pivot.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::MotionMagic, encoderPosition);
}

void Shooter::Periodic(){
  //SmartDashboard 
  frc::SmartDashboard::PutNumber("Shooter/Flywheel power", m_leadMotor.Get()); 
  frc::SmartDashboard::PutNumber("Shooter/Pivot power", m_pivot.Get());
  frc::SmartDashboard::PutNumber("Shooter/Pivot encoder", m_pivot.GetSelectedSensorPosition());

  UpdateVisualization();
}

units::degree_t Shooter::DistanceToAngle(units::meter_t distance) {
  return 30_deg; // temporary value until math is worked out.
}

double Shooter::ToTalonUnits(const frc::Rotation2d &rotation) {
  units::radian_t currentHeading =
      m_pivot.GetSelectedSensorPosition() / ShooterConstants::kAngleToSensor;
  // Puts the rotation in the correct scope for the incremental encoder.
  return (frc::AngleModulus(rotation.Radians() - currentHeading) +
          currentHeading) *
         ShooterConstants::kAngleToSensor;
}

units::radian_t Shooter::GetAnglePivot() {
  // auto offset = ShooterConstants:: kOffset;
  

  // fmt::print("motor encoder: {}\n", m_motor.GetSelectedSensorPosition());

  return (units::radian_t{m_pivot.GetSelectedSensorPosition() *
                             ShooterConstants::kPivotEncoderDistancePerCount} +
         ShooterConstants::kOffset);
   //return 0_rad;
}

// frc2::CommandPtr Shooter::ShooterCommand(std::function<double()> flywheelInput, std::function<units::degree_t()> pivotAngle) {
//   return frc2::cmd::Parallel(
//     FlywheelCommand(flywheelInput),
//     PivotAngleCommand(pivotAngle)
//   );
// }
frc2::CommandPtr Shooter::ShooterCommand(std::function<double()> flywheelInput, std::function<units::meter_t()> calculateDistance) {
    return frc2::cmd::Parallel(
        FlywheelCommand(flywheelInput),
        AimSubwoofer(calculateDistance())
    );
}


frc2::CommandPtr Shooter::FlywheelCommand(std::function<double()> controllerInput) {
  return frc2::cmd::Run(
    [this, controllerInput] { 
      m_leadMotor.Set((controllerInput() * controllerInput()) / 2.0); 
    
      // frc::SmartDashboard::PutNumber("Shooter/Flywheel output", controllerInput());
    }, {}
  );
}

frc2::CommandPtr Shooter::PivotAngleCommand(std::function<units::degree_t()> angle) {
  return frc2::cmd::Run(
    [this, angle] () {
      SetPivotMotor(ToTalonUnits(angle()));
      frc::SmartDashboard::PutNumber("Shooter/Pivot Angle Goal", angle().value());
      frc::SmartDashboard::PutNumber("Shooter/Pivot Encoder Goal", ToTalonUnits(angle()));
      //fmt::print("inside pivot angle command {}\n", angle());
    }, {this}
  );
}


frc2::CommandPtr Shooter::PivotAngleDistanceCommand(units::meter_t distance) {
  return frc2::cmd::RunOnce(
    [this, distance] () {
      SetPivotMotor(ToTalonUnits(DistanceToAngle(distance)));
    }, {this}
  );
}

frc2::CommandPtr Shooter::AimSubwoofer(units::meter_t distance) {
  return frc2::cmd::RunOnce(
    [this, distance] () {
      float angle = KevensCoolEquasion(distance.to<double>()); 
      units::radian_t angle_radians(angle); 
      frc::Rotation2d rotation(angle_radians); 
      SetPivotMotor(ToTalonUnits(rotation)); 
    }, {this}
  );
}



// ************************ SIMULATION *****************************
void Shooter::SimulationPeriodic()
{
  using namespace ShooterConstants;
  if (!m_sim_state) return;

 
  // Simulate main shooter motor
  units::volt_t applied_voltage{
    m_sim_state->m_leadShooterSim.GetDouble("Applied Output").Get()};

  m_sim_state->m_shooterModel.SetInputVoltage(applied_voltage);
  m_sim_state->m_shooterModel.Update(20_ms);
  m_sim_state->m_leadShooterSim.GetDouble("Velocity").Set(
    m_sim_state->m_shooterModel.GetAngularVelocity()
      .convert<units::revolutions_per_minute>().value()
  );
  m_sim_state->m_followerShooterSim.GetDouble("Velocity").Set(
    m_sim_state->m_shooterModel.GetAngularVelocity()
      .convert<units::revolutions_per_minute>().value()
  );
  m_sim_state->m_leadShooterSim.GetDouble("Motor Current").Set(
    m_sim_state->m_shooterModel.GetCurrentDraw().value()
  );

  // Simulate arm
  m_sim_state->m_aimMotorSim.SetBusVoltage(
    frc::RobotController::GetBatteryVoltage().value());
  
  m_sim_state->m_armModel.SetInputVoltage(
    -units::volt_t{m_sim_state->m_aimMotorSim.GetMotorOutputLeadVoltage()}
  );
  m_sim_state->m_armModel.Update(20_ms);

  const units::degree_t arm_angle{m_sim_state->m_armModel.GetAngle()};
  const auto sensor_angle_reading = 
    kMinAimSensor + kAngleToSensor * (arm_angle - kMinAngle);
  m_sim_state->m_aimMotorSim.SetAnalogPosition(sensor_angle_reading);

  // Talon expects speed in terms of "sensor units per 100ms"
  // hence multiplying by 100_ms
  const units::degrees_per_second_t arm_speed{m_sim_state->m_armModel.GetVelocity()};
  const auto sensor_speed_reading = arm_speed * kAngleToSensor * 100_ms;
  m_sim_state->m_aimMotorSim.SetAnalogVelocity(sensor_speed_reading);
  m_sim_state->m_aimMotorSim.SetLimitFwd(
    m_sim_state->m_armModel.HasHitUpperLimit()
  );
  m_sim_state->m_aimMotorSim.SetLimitRev(
    m_sim_state->m_armModel.HasHitLowerLimit()
  );
  m_sim_state->m_aimMotorSim.SetSupplyCurrent(
    m_sim_state->m_armModel.GetCurrentDraw().value()
  );

  frc::SmartDashboard::PutNumber("Shooter/Analog Position", sensor_angle_reading.value());
  frc::SmartDashboard::PutNumber("Shooter/Analog Velocity", sensor_speed_reading.value());
}
