#include "subsystems/SwerveModule.h"

#include <frc/MathUtil.h>
#include <frc/RobotController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <ctre/phoenix6/controls/VelocityDutyCycle.hpp>
#include <ctre/phoenix6/controls/PositionDutyCycle.hpp>

#include <frc/simulation/FlywheelSim.h>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>

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
    m_driveSim(std::move(module.m_driveMotor.GetSimState())),
    m_steerSim(std::move(module.m_steerMotor.GetSimState())),
    m_encoderSim(std::move(module.m_absoluteEncoder.GetSimState())),
    m_wheelModel(frc::DCMotor::Falcon500(1), ModuleConstants::kDriveEncoderReduction, ModuleConstants::kWheelMoment),
    m_swivelModel(frc::DCMotor::Falcon500(1), ModuleConstants::kSteerGearReduction, ModuleConstants::kSteerMoment)
  {}

  void update();

private:
  // hooks to hardware abstraction layer

  ctre::phoenix6::sim::TalonFXSimState m_driveSim, m_steerSim;
  ctre::phoenix6::sim::CANcoderSimState m_encoderSim;

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
  m_driveMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration());
  m_steerMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration());

  //m_steerMotor.SetSelectedSensorPosition(0);

  m_driveMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  m_steerMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);


  ctre::phoenix6::configs::OpenLoopRampsConfigs driveOpenLoopConfigs{};
  driveOpenLoopConfigs.DutyCycleOpenLoopRampPeriod = ModuleConstants::kMotorRampRate;
  driveOpenLoopConfigs.VoltageOpenLoopRampPeriod = ModuleConstants::kMotorRampRate;
  driveOpenLoopConfigs.TorqueOpenLoopRampPeriod = ModuleConstants::kMotorRampRate;

  m_driveMotor.GetConfigurator().Apply(driveOpenLoopConfigs, 50_ms);

  ctre::phoenix6::configs::ClosedLoopRampsConfigs driveClosedLoopConfigs{};
  driveClosedLoopConfigs.DutyCycleClosedLoopRampPeriod = ModuleConstants::kMotorRampRate;
  driveClosedLoopConfigs.VoltageClosedLoopRampPeriod = ModuleConstants::kMotorRampRate;
  driveClosedLoopConfigs.TorqueClosedLoopRampPeriod = ModuleConstants::kMotorRampRate;

  m_driveMotor.GetConfigurator().Apply(driveClosedLoopConfigs, 50_ms);

  // Hopefully prevents brownouts.
  ctre::phoenix6::configs::CurrentLimitsConfigs driveCurrentConfigs{};
  driveCurrentConfigs.StatorCurrentLimit = ModuleConstants::kDriveMotorCurrentLimit;
  driveCurrentConfigs.StatorCurrentLimitEnable = true;

  m_driveMotor.GetConfigurator().Apply(driveCurrentConfigs, 50_ms);

  ctre::phoenix6::configs::CurrentLimitsConfigs steerCurrentConfigs{};
  steerCurrentConfigs.StatorCurrentLimit = ModuleConstants::kSteerMotorCurrentLimit;
  steerCurrentConfigs.StatorCurrentLimitEnable = true;

  m_steerMotor.GetConfigurator().Apply(steerCurrentConfigs, 50_ms);
  
  ctre::phoenix6::configs::Slot0Configs drivePIDConfigs{};
  drivePIDConfigs.kP = driveMotorPIDCoefficients.kP;
  drivePIDConfigs.kI = driveMotorPIDCoefficients.kI;
  drivePIDConfigs.kD = driveMotorPIDCoefficients.kD;
  drivePIDConfigs.kV = 1.0/(ModuleConstants::kPhysicalMaxSpeed / ModuleConstants::kDistanceToRotations * 1_s / 1_tr);

  m_driveMotor.GetConfigurator().Apply(drivePIDConfigs, 50_ms);
  //m_driveMotor.Config_kF(0, driveMotorPIDCoefficients.kFF);
  // really this isn't something that should be arbitrarily tuned, we can calculate it
  // See the documentation on Config_kF


  ctre::phoenix6::configs::Slot0Configs steerPIDConfigs{};
  steerPIDConfigs.kP = steerMotorPIDCoefficients.kP;
  steerPIDConfigs.kI = steerMotorPIDCoefficients.kI;
  steerPIDConfigs.kD = steerMotorPIDCoefficients.kD;
  steerPIDConfigs.kV = 0.0;
  
  m_steerMotor.GetConfigurator().Apply(steerPIDConfigs, 50_ms);

  ctre::phoenix6::configs::ClosedLoopGeneralConfigs steerClosedLoopConfig{};
  steerClosedLoopConfig.ContinuousWrap = true;

  m_steerMotor.GetConfigurator().Apply(steerClosedLoopConfig, 50_ms);
  
  // This automatically scales future setpoints and readings by gear ratio
  ctre::phoenix6::configs::FeedbackConfigs steerFeedbackConfigs{};
  steerFeedbackConfigs.SensorToMechanismRatio = ModuleConstants::kSteerGearReduction;
  m_steerMotor.GetConfigurator().Apply(steerFeedbackConfigs, 50_ms);

  // Home the integrated rotor sensor to the cancoder position
  m_steerMotor.SetPosition(m_absoluteEncoder.GetAbsolutePosition().GetValue());

  // // Connects CANCoder to the steer motor
  // m_steerMotor.ConfigRemoteFeedbackFilter(m_absoluteEncoder, 0);
  // m_steerMotor.ConfigSelectedFeedbackSensor(
  //     ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0, 0, 0);
      
  
  // positive voltage is counter clockwise
  m_steerMotor.SetInverted(true);
  // m_steerMotor.SetInverted(false);
}

SwerveModule::~SwerveModule() {}

units::meter_t SwerveModule::GetModuleDistance() {
  return m_driveMotor.GetPosition().GetValue() * ModuleConstants::kDistanceToRotations;
}

units::meters_per_second_t SwerveModule::GetModuleVelocity() {
  return m_driveMotor.GetVelocity().GetValue() * ModuleConstants::kDistanceToRotations;
}

frc::Rotation2d SwerveModule::GetModuleHeading() {
  return m_absoluteEncoder.GetAbsolutePosition().GetValue().convert<units::degree>();
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

  ctre::phoenix6::controls::VelocityDutyCycle velocityControl{0_tps, 0_tr_per_s_sq, false};
  m_driveMotor.SetControl(velocityControl.WithVelocity(state.speed / ModuleConstants::kDistanceToRotations));
  
  ctre::phoenix6::controls::PositionDutyCycle positionControl{0_tr, 0_tps, false};

  m_steerMotor.SetControl(positionControl.WithPosition(state.angle.Radians()));
  // frc::SmartDashboard::PutNumber(fmt::format("{}/angle", m_name),
  //                                state.angle.Degrees().value());
  // frc::SmartDashboard::PutNumber(fmt::format("{}/talon angle setpoint", m_name),
  //                                ToTalonAngle(state.angle));
  // frc::SmartDashboard::PutNumber(fmt::format("{}/velocity output (mps)", m_name), state.speed.value());
}

// TODO Display things neater on the SmartDashboard.
void SwerveModule::UpdateDashboard() {
  const auto state = GetState();

  auto& steerRotorPosSignal = m_steerMotor.GetPosition();

  auto steerRotorPos = steerRotorPosSignal.GetValue();

  frc::SmartDashboard::PutString(fmt::format("{}/module state", m_name),
                                 fmt::format("{:4f}@{:4f}°",
                                             state.speed.value(),
                                             state.angle.Degrees().value()));
  frc::SmartDashboard::PutNumber(fmt::format("{}/absolute position", m_name),
                                 units::degree_t{GetAbsoluteEncoderPosition()}.value());
  frc::SmartDashboard::PutNumber(fmt::format("{}/steer talon angle", m_name),
                                 steerRotorPos.convert<units::degree>().value());
  // TODO: Fix Error Accumulation.
  // frc::SmartDashboard::PutNumber(fmt::format("{}/steer err accum", m_name),
  //                                m_steerMotor.GetIntegralAccumulator());
  frc::SmartDashboard::PutNumber(fmt::format("{}/velocity state (mps)", m_name), state.speed.value());
  frc::SmartDashboard::PutNumber(fmt::format("{}/drive voltage", m_name), m_driveMotor.GetSupplyVoltage().GetValueAsDouble());
  frc::SmartDashboard::PutNumber(fmt::format("{}/turn voltage", m_name), m_steerMotor.GetSupplyVoltage().GetValueAsDouble());
  frc::SmartDashboard::PutNumber(fmt::format("{}/drive current", m_name), m_driveMotor.GetSupplyCurrent().GetValueAsDouble());
  frc::SmartDashboard::PutNumber(fmt::format("{}/turn current", m_name), m_steerMotor.GetSupplyCurrent().GetValueAsDouble());

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
  // units::radian_t currentHeading =
  //     ModuleConstants::kSteerEncoderDistancePerCount * m_steerMotor.GetSelectedSensorPosition();

  auto& steerRotorPosSignal = m_steerMotor.GetPosition();
  units::radian_t currentHeading = steerRotorPosSignal.GetValue();
  // Puts the rotation in the correct scope for the incremental encoder.
  return (frc::AngleModulus(rotation.Radians() - currentHeading) +
          currentHeading) /
         ModuleConstants::kSteerEncoderDistancePerCount;
}

units::radian_t SwerveModule::GetAbsoluteEncoderPosition() {
  return m_absoluteEncoder.GetAbsolutePosition().GetValue().convert<units::radian>();
}

// Simulation
void SwerveModule::SimulationPeriodic()
{
  if(m_sim_state) m_sim_state->update();
}

void SwerveModuleSim::update()
{
  m_driveSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  m_steerSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

  // Simulate the wheel swiveling
  const auto prev_velocity = m_swivelModel.GetAngularVelocity();
  m_swivelModel.SetInputVoltage(m_steerSim.GetMotorVoltage());
  m_swivelModel.Update(20_ms);
  const auto average_velocity = (prev_velocity + m_swivelModel.GetAngularVelocity())/2;
  // cancoder is on mechanism and is inverted from the falcon's rotor
  m_encoderSim.AddPosition(-average_velocity*20_ms);
  m_steerSim.AddRotorPosition(average_velocity*ModuleConstants::kSteerGearReduction*20_ms);
  m_steerSim.SetRotorVelocity(average_velocity*ModuleConstants::kSteerGearReduction);

  // Simulate the wheel turning (ignoring changes in traction)
  m_wheelModel.SetInputVoltage(m_driveSim.GetMotorVoltage());
  m_wheelModel.Update(20_ms);

  m_driveSim.SetRotorVelocity(m_wheelModel.GetAngularVelocity()*ModuleConstants::kDriveEncoderReduction);
  m_driveSim.AddRotorPosition(m_wheelModel.GetAngularVelocity()*ModuleConstants::kDriveEncoderReduction*20_ms);
}