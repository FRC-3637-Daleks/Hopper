// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/simulation/FlywheelSim.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc/simulation/DIOSim.h>

class ShooterSimulation
{
public:
  ShooterSimulation(Shooter &shooter):
    m_leadShooterSim{"SPARK MAX ", ShooterConstants::kShooterLeadMotorPort},
    m_followerShooterSim{"SPARK MAX ", ShooterConstants::kShooterFollowerMotorPort},
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

  m_pivot.SetSelectedSensorPosition(0,ShooterConstants::kPIDLoopIdx,ShooterConstants::kTimeoutMs);

  m_pivot.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::Analog,ShooterConstants::kPIDLoopIdx,ShooterConstants::kTimeoutMs);

  m_pivot.SetSensorPhase(true);

  m_pivot.ConfigNominalOutputForward(0,ShooterConstants::kTimeoutMs);
  m_pivot.ConfigNominalOutputReverse(0,ShooterConstants::kTimeoutMs);
  m_pivot.ConfigPeakOutputForward(1.0,ShooterConstants::kTimeoutMs);
  m_pivot.ConfigPeakOutputReverse(1.0,ShooterConstants::kTimeoutMs);
  
  m_pivot.Config_kF(ShooterConstants::kPIDLoopIdx, 0.0,ShooterConstants::kTimeoutMs);
  m_pivot.Config_kP(ShooterConstants::kPIDLoopIdx, 0.0,ShooterConstants::kTimeoutMs);
  m_pivot.Config_kI(ShooterConstants::kPIDLoopIdx, 0.0,ShooterConstants::kTimeoutMs);
  m_pivot.Config_kD(ShooterConstants::kPIDLoopIdx, 0.0,ShooterConstants::kTimeoutMs);
//Motors following + leading

  m_followMotor.Follow(m_leadMotor);
  
//Configure for use 

  


}

Shooter::~Shooter() {}

//Runs both shooting motors
void Shooter::runShootMotor(){

//Starts
   m_leadMotor.SetVoltage(1.0_V);
   
}

//Stops both shooting motors
void Shooter::stopShootMotor(){

//Stops
  m_leadMotor.StopMotor();

}

//Runs pivoting motor
void Shooter::runTalonMotor(){
//Runs
m_pivot.SetVoltage(1.0_V);

}

//Stop pivoting motor
void Shooter::stopTalonMotor(){
//Stops
 m_pivot.StopMotor();

}

void Shooter::Periodic(){

//SmartDashboard 
frc::SmartDashboard::PutNumber("motor power", m_leadMotor.Get()); 

}

frc2::CommandPtr Shooter::IntakeCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  // return StartEnd([this] { intake.Set(0.5); }, 
  //                 [this] { intake.Set(0.0); });
  return frc2::cmd::Idle();
}

frc2::CommandPtr Shooter::FlywheelCommand( double controllerInput ) {
  return Run([this, &controllerInput] 
             { m_leadMotor.Set(controllerInput); });
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
    units::volt_t{m_sim_state->m_aimMotorSim.GetMotorOutputLeadVoltage()}
  );
  m_sim_state->m_armModel.Update(20_ms);

  const units::degree_t arm_angle{m_sim_state->m_armModel.GetAngle()};
  const auto sensor_angle_reading = 
    kMinAimSensor + kAngleToSensor*(arm_angle - kMinAngle);
  m_sim_state->m_aimMotorSim.SetAnalogPosition(sensor_angle_reading);

  // Talon expects speed in terms of "sensor units per 100ms"
  // hence multiplying by 100_ms
  const units::degrees_per_second_t arm_speed{m_sim_state->m_armModel.GetVelocity()};
  const auto sensor_speed_reading = arm_speed*kAngleToSensor*100_ms;
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
}
