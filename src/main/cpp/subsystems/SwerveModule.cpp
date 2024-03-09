#include "subsystems/SwerveModule.h"

#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/controls/PositionDutyCycle.hpp>
#include <ctre/phoenix6/controls/VelocityDutyCycle.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <frc/DataLogManager.h>
#include <frc/MathUtil.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <frc/simulation/FlywheelSim.h>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/moment_of_inertia.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <iostream>
#include <random>

using namespace ModuleConstants;

class SwerveModuleSim {
public:
  SwerveModuleSim(SwerveModule &module)
      : m_driveSim(std::move(module.m_driveMotor.GetSimState())),
        m_steerSim(std::move(module.m_steerMotor.GetSimState())),
        m_encoderSim(std::move(module.m_absoluteEncoder.GetSimState())),
        m_wheelModel(frc::DCMotor::Falcon500(1),
                     ModuleConstants::kDriveEncoderReduction,
                     ModuleConstants::kWheelMoment),
        m_swivelModel(frc::DCMotor::Falcon500(1),
                      ModuleConstants::kSteerGearReduction,
                      ModuleConstants::kSteerMoment) {
    static std::random_device rng;
    std::uniform_real_distribution dist(-0.5, 0.5);

    // randomize starting positions
    m_encoderSim.SetRawPosition(units::turn_t{dist(rng)});
  }

  void update();

private:
  // hooks to hardware abstraction layer

  ctre::phoenix6::sim::TalonFXSimState m_driveSim, m_steerSim;
  ctre::phoenix6::sim::CANcoderSimState m_encoderSim;

  // tracks the simulation state for each wheel
  frc::sim::FlywheelSim m_wheelModel, m_swivelModel;
};

SwerveModule::SwerveModule(const std::string name, const int driveMotorId,
                           const int steerMotorId, const int absoluteEncoderId,
                           const PIDCoefficients driveMotorPIDCoefficients,
                           const PIDCoefficients steerMotorPIDCoefficients)
    : m_name{name}, m_driveMotor(driveMotorId), m_steerMotor(steerMotorId),
      m_absoluteEncoder(absoluteEncoderId),
      m_drivePosition(m_driveMotor.GetPosition()),
      m_driveVelocity(m_driveMotor.GetVelocity()),
      m_steerPosition(m_steerMotor.GetPosition()), //< FusedCANCoder
      m_steerVelocity(m_steerMotor.GetVelocity()),
      m_sim_state(new SwerveModuleSim(*this)) {

  ctre::phoenix6::configs::TalonFXConfiguration steerConfig, driveConfig;

  ctre::phoenix6::configs::MotorOutputConfigs steerOutputConfigs;
  steerOutputConfigs.WithNeutralMode(
      ctre::phoenix6::signals::NeutralModeValue::Brake);
  steerOutputConfigs.WithInverted(true);
  steerConfig.WithMotorOutput(steerOutputConfigs);

  ctre::phoenix6::configs::MotorOutputConfigs driveOutputConfigs;
  driveOutputConfigs.WithNeutralMode(
      ctre::phoenix6::signals::NeutralModeValue::Brake);
  driveOutputConfigs.WithDutyCycleNeutralDeadband(NeutralDeadBand);
  driveConfig.WithMotorOutput(driveOutputConfigs);

  ctre::phoenix6::configs::OpenLoopRampsConfigs driveOpenLoopConfigs{};
  driveOpenLoopConfigs.DutyCycleOpenLoopRampPeriod = kMotorRampRate;
  driveOpenLoopConfigs.VoltageOpenLoopRampPeriod = kMotorRampRate;
  driveOpenLoopConfigs.TorqueOpenLoopRampPeriod = kMotorRampRate;
  driveConfig.WithOpenLoopRamps(driveOpenLoopConfigs);

  ctre::phoenix6::configs::ClosedLoopRampsConfigs driveClosedLoopConfigs{};
  driveClosedLoopConfigs.DutyCycleClosedLoopRampPeriod = kMotorRampRate;
  driveClosedLoopConfigs.VoltageClosedLoopRampPeriod = kMotorRampRate;
  driveClosedLoopConfigs.TorqueClosedLoopRampPeriod = kMotorRampRate;
  driveConfig.WithClosedLoopRamps(driveClosedLoopConfigs);

  // Hopefully prevents brownouts.
  ctre::phoenix6::configs::CurrentLimitsConfigs driveCurrentConfigs{};
  driveCurrentConfigs.SupplyCurrentLimitEnable = true;
  driveCurrentConfigs.SupplyCurrentLimit = kDriveMotorCurrentLimit;
  driveCurrentConfigs.SupplyCurrentThreshold = kDriveMotorCurrentLimit;
  driveCurrentConfigs.SupplyTimeThreshold =
      kCurrentLimitPeriod.convert<units::second>().value();
  driveConfig.WithCurrentLimits(driveCurrentConfigs);

  ctre::phoenix6::configs::CurrentLimitsConfigs steerCurrentConfigs{};
  steerCurrentConfigs.SupplyCurrentLimitEnable = true;
  steerCurrentConfigs.SupplyCurrentLimit = kSteerMotorCurrentLimit;
  steerCurrentConfigs.SupplyCurrentThreshold = kSteerMotorCurrentLimit;
  steerCurrentConfigs.SupplyTimeThreshold =
      kCurrentLimitPeriod.convert<units::second>().value();
  steerConfig.WithCurrentLimits(steerCurrentConfigs);

  ctre::phoenix6::configs::Slot0Configs drivePIDConfigs{};
  drivePIDConfigs.kP = driveMotorPIDCoefficients.kP;
  drivePIDConfigs.kI = driveMotorPIDCoefficients.kI;
  drivePIDConfigs.kD = driveMotorPIDCoefficients.kD;
  drivePIDConfigs.kV =
      1.0 / (kPhysicalMaxSpeed / kDistanceToRotations * 1_s / 1_tr);
  driveConfig.WithSlot0(drivePIDConfigs);

  ctre::phoenix6::configs::Slot0Configs steerPIDConfigs{};
  steerPIDConfigs.kP = steerMotorPIDCoefficients.kP;
  steerPIDConfigs.kI = steerMotorPIDCoefficients.kI;
  steerPIDConfigs.kD = steerMotorPIDCoefficients.kD;
  steerPIDConfigs.kV = 0.0;
  steerConfig.WithSlot0(steerPIDConfigs);

  ctre::phoenix6::configs::ClosedLoopGeneralConfigs steerClosedLoopConfig{};
  steerClosedLoopConfig.ContinuousWrap = true;
  steerConfig.WithClosedLoopGeneral(steerClosedLoopConfig);

  ctre::phoenix6::configs::FeedbackConfigs steerFeedbackConfigs{};
  steerFeedbackConfigs.FeedbackSensorSource =
      ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
  steerFeedbackConfigs.FeedbackRemoteSensorID = m_absoluteEncoder.GetDeviceID();
  // This automatically scales future setpoints and readings by gear ratio
  steerFeedbackConfigs.RotorToSensorRatio = kSteerGearReduction;
  steerFeedbackConfigs.SensorToMechanismRatio = 1.0;
  steerConfig.WithFeedback(steerFeedbackConfigs);

  int retries = 4;
  while (auto ret = m_driveMotor.GetConfigurator().Apply(driveConfig, 500_ms)) {
    if (retries-- == 0) {
      // when ret is non-zero, that means there's an error
      std::cerr << "ERROR Applying Drive Motor Configs for " << m_name
                << std::endl;
      std::cerr << "Talon ID: " << driveMotorId << ", Error: " << ret
                << std::endl;
      break;
    }
  }

  retries = 4;
  while (auto ret = m_steerMotor.GetConfigurator().Apply(steerConfig, 500_ms)) {
    if (retries-- == 0) {
      std::cerr << "ERROR Applying Steer Motor Configs for " << m_name
                << std::endl;
      std::cerr << "Talon ID: " << steerMotorId << ", Error: " << ret
                << std::endl;
      break;
    }
  }

  frc::DataLogManager::Log(
      fmt::format("Finished initializing {} swerve module", m_name));
}

SwerveModule::~SwerveModule() {}

void SwerveModule::RefreshSignals() {
  /* Refreshes all this modules signals at once.
   * This should improve performance
   */
  ctre::phoenix6::BaseStatusSignal::RefreshAll(
      m_drivePosition, m_driveVelocity, m_steerPosition, m_steerVelocity);
}

units::meter_t SwerveModule::GetModuleDistance() {
  const auto position =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          m_drivePosition, m_driveVelocity);
  return position * kDistanceToRotations;
}

units::meters_per_second_t SwerveModule::GetModuleVelocity() {
  return m_driveVelocity.GetValue() * kDistanceToRotations;
}

frc::Rotation2d SwerveModule::GetModuleHeading() {
  const auto position =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          m_steerPosition, m_steerVelocity);
  return position.convert<units::degree>();
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return {GetModuleDistance(), GetModuleHeading()};
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {GetModuleVelocity(), GetModuleHeading()};
}

void SwerveModule::CoastMode(bool coast) {
  if (coast) {
    m_steerMotor.SetNeutralMode(
        ctre::phoenix6::signals::NeutralModeValue::Coast);
    m_driveMotor.SetNeutralMode(
        ctre::phoenix6::signals::NeutralModeValue::Coast);
  } else {
    m_steerMotor.SetNeutralMode(
        ctre::phoenix6::signals::NeutralModeValue::Brake);
    m_driveMotor.SetNeutralMode(
        ctre::phoenix6::signals::NeutralModeValue::Brake);
  }
}

void SwerveModule::SetEncoderOffset() {
  ctre::phoenix6::configs::MagnetSensorConfigs magConfig;
  double position = m_absoluteEncoder.GetAbsolutePosition().GetValue().value();
  magConfig.WithMagnetOffset(-position);
  magConfig.WithAbsoluteSensorRange(
      ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf);
  magConfig.WithSensorDirection(
      ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive);

  m_absoluteEncoder.GetConfigurator().Apply(magConfig, 50_ms);

  SyncEncoders();
}

void SwerveModule::ZeroAbsEncoders() {
  ctre::phoenix6::configs::MagnetSensorConfigs magConfig;
  magConfig.WithMagnetOffset(0);
  magConfig.WithAbsoluteSensorRange(
      ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf);
  magConfig.WithSensorDirection(
      ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive);

  m_absoluteEncoder.GetConfigurator().Apply(magConfig, 50_ms);
}

void SwerveModule::SyncEncoders() {
  fmt::print("inside sync encoder*******");

  m_steerMotor.SetPosition(m_absoluteEncoder.GetAbsolutePosition().GetValue());
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState &referenceState) {
  // Optimize the reference state to prevent the module turning >90 degrees.
  const auto state =
      frc::SwerveModuleState::Optimize(referenceState, GetModuleHeading());

  ctre::phoenix6::controls::VelocityDutyCycle velocityControl{
      0_tps, 0_tr_per_s_sq, true};
  m_driveMotor.SetControl(
      velocityControl.WithVelocity(state.speed / kDistanceToRotations));

  ctre::phoenix6::controls::PositionDutyCycle positionControl{0_tr, 0_tps,
                                                              false};

  m_steerMotor.SetControl(positionControl.WithPosition(state.angle.Radians()));
}

// TODO Display things neater on the SmartDashboard.
void SwerveModule::UpdateDashboard() {
  const auto state = GetState();
  frc::SmartDashboard::PutNumber(
      fmt::format("Swerve/{}/heading (degrees)", m_name),
      state.angle.Degrees().value());
  frc::SmartDashboard::PutNumber(fmt::format("Swerve/{}/speed (mps)", m_name),
                                 state.speed.convert<units::mps>().value());
}

units::radian_t SwerveModule::GetAbsoluteEncoderPosition() {
  return m_absoluteEncoder.GetAbsolutePosition()
      .GetValue()
      .convert<units::radian>();
}

// Simulation
void SwerveModule::SimulationPeriodic() {
  if (m_sim_state)
    m_sim_state->update();
}

void SwerveModuleSim::update() {
  m_driveSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  m_steerSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

  // Simulate the wheel swiveling
  const auto prev_velocity = m_swivelModel.GetAngularVelocity();
  m_swivelModel.SetInputVoltage(m_steerSim.GetMotorVoltage());
  m_swivelModel.Update(20_ms);
  const auto average_velocity =
      (prev_velocity + m_swivelModel.GetAngularVelocity()) / 2;
  // cancoder is on mechanism and is inverted from the falcon's rotor
  m_encoderSim.AddPosition(-average_velocity * 20_ms);
  m_steerSim.AddRotorPosition(average_velocity * kSteerGearReduction * 20_ms);
  m_steerSim.SetRotorVelocity(average_velocity * kSteerGearReduction);

  // Simulate the wheel turning (ignoring changes in traction)
  m_wheelModel.SetInputVoltage(m_driveSim.GetMotorVoltage());
  m_wheelModel.Update(20_ms);

  m_driveSim.SetRotorVelocity(m_wheelModel.GetAngularVelocity() *
                              kDriveEncoderReduction);
  m_driveSim.AddRotorPosition(m_wheelModel.GetAngularVelocity() *
                              kDriveEncoderReduction * 20_ms);
}