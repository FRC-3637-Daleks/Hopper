#include "subsystems/SwerveModule.h"

#include <frc/MathUtil.h>
#include <frc/RobotController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/simulation/FlywheelSim.h>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/moment_of_inertia.h>

#include <iostream>

class SwerveModuleSim
{
public:
  SwerveModuleSim(SwerveModule& module):
    m_driveSim(module.m_driveMotor.GetSimCollection()),
    m_steerSim(module.m_steerMotor.GetSimCollection()),
    m_encoderSim(module.m_absoluteEncoder.GetSimCollection()),
    m_wheelModel(frc::DCMotor::Falcon500(1), ModuleConstants::kDriveEncoderReduction, ModuleConstants::kWheelMoment),
    m_swivelModel(frc::DCMotor::Falcon500(1), ModuleConstants::kSteerGearReduction, ModuleConstants::kSteerMoment)
  {}

  void update();

private:
  // hooks to hardware abstraction layer
  ctre::phoenix::motorcontrol::TalonFXSimCollection m_driveSim, m_steerSim;
  ctre::phoenix::sensors::CANCoderSimCollection m_encoderSim;

  // tracks the simulation state for each wheel
  frc::sim::FlywheelSim m_wheelModel, m_swivelModel;
};

SwerveModule::SwerveModule(const std::string name, const int driveMotorId,
                           const int steerMotorId,
                           const int absoluteEncoderId,
                           const double absoluteEncoderOffset,
                           const PIDCoefficients driveMotorPIDCoefficients,
                           const PIDCoefficients steerMotorPIDCoefficients)
    : m_name{name},
      m_driveMotor(driveMotorId),
      m_steerMotor(steerMotorId),
      // Have the absolute encoder return radian values.
      m_absoluteEncoder(absoluteEncoderId),
      m_sim_state(new SwerveModuleSim(*this)) {
  // Reset the drive and steer motor controllers to their default settings,
  // then configure them for use.
  m_driveMotor.ConfigFactoryDefault();
  m_steerMotor.ConfigFactoryDefault();
  m_steerMotor.SetSelectedSensorPosition(0);
  m_driveMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_steerMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_driveMotor.ConfigClosedloopRamp(ModuleConstants::kMotorRampRate);
  m_driveMotor.ConfigOpenloopRamp(ModuleConstants::kMotorRampRate);
  // Hopefully prevents brownouts.
  m_driveMotor.ConfigStatorCurrentLimit({true, ModuleConstants::kDriveMotorCurrentLimit, ModuleConstants::kDriveMotorCurrentLimit, 1.0});
  m_steerMotor.ConfigStatorCurrentLimit({true, ModuleConstants::kSteerMotorCurrentLimit, ModuleConstants::kSteerMotorCurrentLimit, 1.0});
  
  m_driveMotor.Config_kP(0, driveMotorPIDCoefficients.kP);
  m_driveMotor.Config_kI(0, driveMotorPIDCoefficients.kI);
  m_driveMotor.Config_kD(0, driveMotorPIDCoefficients.kD);
  //m_driveMotor.Config_kF(0, driveMotorPIDCoefficients.kFF);
  // really this isn't something that should be arbitrarily tuned, we can calculate it
  // See the documentation on Config_kF
  m_driveMotor.Config_kF(0, 1023/ToTalonVelocity(ModuleConstants::kPhysicalMaxSpeed));
  m_steerMotor.Config_kP(0, steerMotorPIDCoefficients.kP);
  m_steerMotor.Config_kI(0, steerMotorPIDCoefficients.kI);
  m_steerMotor.Config_kD(0, steerMotorPIDCoefficients.kD);

  m_absoluteEncoder.ConfigSensorInitializationStrategy(
    ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition
  );

  m_absoluteEncoder.ConfigFeedbackCoefficient(
    ModuleConstants::kSteerEncoderDistancePerCount.value(), "rad",
    ctre::phoenix::sensors::SensorTimeBase::PerSecond);

  // Connects CANCoder to the steer motor
  m_steerMotor.ConfigRemoteFeedbackFilter(m_absoluteEncoder, 0);
  m_steerMotor.ConfigSelectedFeedbackSensor(
      ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0, 0, 0);
  
  // positive voltage is counter clockwise
  m_steerMotor.SetInverted(true);
  m_steerMotor.SetSensorPhase(true);
  m_absoluteEncoder.ConfigSensorDirection(false);
  // m_steerMotor.SetInverted(false);
}

SwerveModule::~SwerveModule() {}

units::meter_t SwerveModule::GetModuleDistance() {
  return ModuleConstants::kDriveEncoderDistancePerRevolution * m_driveMotor.GetSelectedSensorPosition() / ModuleConstants::kDriveEncoderCPR;
}

units::meters_per_second_t SwerveModule::GetModuleVelocity() {
  return ModuleConstants::kDriveEncoderDistancePerRevolution * m_driveMotor.GetSelectedSensorVelocity() / ModuleConstants::kDriveEncoderCPR / 100_ms;
}

frc::Rotation2d SwerveModule::GetModuleHeading() {
  return frc::AngleModulus(m_absoluteEncoder.GetAbsolutePosition()*1_rad);
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return {GetModuleDistance(), GetModuleHeading()};
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {GetModuleVelocity(), GetModuleHeading()};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState &referenceState) {
  // Optimize the reference state to prevent the module turning >90 degrees.
  const auto state =
      frc::SwerveModuleState::Optimize(referenceState, GetModuleHeading());

  m_driveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, 
                    ToTalonVelocity(state.speed));
  //m_driveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 
  //                 state.speed/kPhysicalMaxSpeed);

  m_steerMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
                   ToTalonAngle(state.angle));
  // frc::SmartDashboard::PutNumber(fmt::format("{}/angle", m_name),
  //                                state.angle.Degrees().value());
  // frc::SmartDashboard::PutNumber(fmt::format("{}/talon angle setpoint", m_name),
  //                                ToTalonAngle(state.angle));
  // frc::SmartDashboard::PutNumber(fmt::format("{}/velocity output (mps)", m_name), state.speed.value());
}

// TODO Display things neater on the SmartDashboard.
void SwerveModule::UpdateDashboard() {
  const auto state = GetState();
  frc::SmartDashboard::PutString(fmt::format("{}/module state", m_name),
                                 fmt::format("{:4f}@{:4f}°",
                                             state.speed.value(),
                                             state.angle.Degrees().value()));
  frc::SmartDashboard::PutNumber(fmt::format("{}/absolute position", m_name),
                                 units::degree_t{GetAbsoluteEncoderPosition()}.value());
  frc::SmartDashboard::PutNumber(fmt::format("{}/steer talon angle", m_name),
                                 m_steerMotor.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber(fmt::format("{}/steer err accum", m_name),
                                 m_steerMotor.GetIntegralAccumulator());
  frc::SmartDashboard::PutNumber(fmt::format("{}/velocity state (mps)", m_name), state.speed.value());
  frc::SmartDashboard::PutNumber(fmt::format("{}/drive voltage", m_name), m_driveMotor.GetBusVoltage());
  frc::SmartDashboard::PutNumber(fmt::format("{}/turn voltage", m_name), m_steerMotor.GetBusVoltage());
  frc::SmartDashboard::PutNumber(fmt::format("{}/drive current", m_name), m_driveMotor.GetOutputCurrent());
  frc::SmartDashboard::PutNumber(fmt::format("{}/turn current", m_name), m_steerMotor.GetOutputCurrent());

  frc::SmartDashboard::PutNumber(fmt::format("{}/angle", m_name),
                                 state.angle.Degrees().value());
  frc::SmartDashboard::PutNumber(fmt::format("{}/talon angle setpoint", m_name),
                                 ToTalonAngle(state.angle));
  frc::SmartDashboard::PutNumber(fmt::format("{}/velocity output (mps)", m_name), state.speed.value());
}

double SwerveModule::ToTalonVelocity(units::meters_per_second_t speed) {
  // encoder ticks per 100ms
  return speed * 100_ms / ModuleConstants::kDriveEncoderDistancePerRevolution * ModuleConstants::kDriveEncoderCPR;
}

double SwerveModule::ToTalonAngle(const frc::Rotation2d &rotation) {
  units::radian_t currentHeading =
      ModuleConstants::kSteerEncoderDistancePerCount * m_steerMotor.GetSelectedSensorPosition();
  // Puts the rotation in the correct scope for the incremental encoder.
  return (frc::AngleModulus(rotation.Radians() - currentHeading) +
          currentHeading) /
         ModuleConstants::kSteerEncoderDistancePerCount;
}

units::radian_t SwerveModule::GetAbsoluteEncoderPosition() {
  return frc::AngleModulus(m_absoluteEncoder.GetAbsolutePosition() * 1_rad);
}

// Simulation
void SwerveModule::SimulationPeriodic()
{
  if(m_sim_state) m_sim_state->update();
}

void SwerveModuleSim::update()
{
  m_driveSim.SetBusVoltage(frc::RobotController::GetBatteryVoltage().value());
  m_steerSim.SetBusVoltage(frc::RobotController::GetBatteryVoltage().value());

  // Simulate the wheel swiveling
  const auto prev_velocity = m_swivelModel.GetAngularVelocity();
  // invert swivel output as per real robot
  m_swivelModel.SetInputVoltage(-m_steerSim.GetMotorOutputLeadVoltage()*1_V);
  m_swivelModel.Update(20_ms);
  const auto average_velocity = (prev_velocity + m_swivelModel.GetAngularVelocity())/2;
  m_encoderSim.AddPosition(average_velocity/(2_rad*std::numbers::pi)*20_ms*ModuleConstants::kSteerEncoderCPR);

  // Simulate the wheel turning (ignoring changes in traction)
  m_wheelModel.SetInputVoltage(m_driveSim.GetMotorOutputLeadVoltage()*1_V);
  m_wheelModel.Update(20_ms);

  const auto tick_speed = m_wheelModel.GetAngularVelocity()/(2_rad*std::numbers::pi)*ModuleConstants::kDriveEncoderCPR;
  m_driveSim.SetIntegratedSensorVelocity(tick_speed*100_ms);
  m_driveSim.AddIntegratedSensorPosition(tick_speed*20_ms);
}