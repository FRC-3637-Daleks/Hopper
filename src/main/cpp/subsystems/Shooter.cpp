// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

#include <frc2/command/Commands.h>

#include <frc/DataLogManager.h>
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/DriverStation.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/simulation/DIOSim.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc/simulation/SingleJointedArmSim.h>

class ShooterSimulation {
public:
  ShooterSimulation(Shooter &shooter)
      : m_leadShooterSim{"SPARK Flex ",
                         ShooterConstants::kFlywheelLeadMotorPort},
        m_followerShooterSim{"SPARK Flex ",
                             ShooterConstants::kFlywheelFollowMotorPort},
        m_aimMotorSim{shooter.m_pivot.GetSimCollection()},
        m_shooterModel{frc::DCMotor::NeoVortex(1), 1,
                       ShooterConstants::kWheelMoment},
        m_armModel{ShooterConstants::kWindowMotor,
                   ShooterConstants::kArmGearing,
                   ShooterConstants::kArmMoment,
                   ShooterConstants::kArmRadius,
                   ShooterConstants::kMinAngle,
                   ShooterConstants::kMaxAngle,
                   false,
                   ShooterConstants::kMaxAngle} {}

public:
  frc::sim::SimDeviceSim m_leadShooterSim, m_followerShooterSim;
  ctre::phoenix::motorcontrol::TalonSRXSimCollection &m_aimMotorSim;

  // models the physics of the components
  frc::sim::FlywheelSim m_shooterModel;
  frc::sim::SingleJointedArmSim m_armModel;
};

Shooter::Shooter() : m_sim_state(new ShooterSimulation(*this)) {
  // Implementation of subsystem constructor goes here.
  // Needs arguments to work between power cycles!!
  // Resets config perameters
  m_followMotor.RestoreFactoryDefaults();
  m_leadMotor.RestoreFactoryDefaults();
  m_pivot.ConfigFactoryDefault();

  m_pivot.SetSelectedSensorPosition(0, ShooterConstants::kPIDLoopIdx,
                                    ShooterConstants::kTimeoutMs);

  m_pivot.ConfigSelectedFeedbackSensor(
      ctre::phoenix::motorcontrol::FeedbackDevice::Analog,
      ShooterConstants::kPIDLoopIdx, ShooterConstants::kTimeoutMs);

  m_pivot.SetSensorPhase(false);
  m_pivot.SetInverted(false);

  m_pivot.ConfigNominalOutputForward(0, ShooterConstants::kTimeoutMs);
  m_pivot.ConfigNominalOutputReverse(0, ShooterConstants::kTimeoutMs);
  m_pivot.ConfigPeakOutputForward(1.0, ShooterConstants::kTimeoutMs);
  m_pivot.ConfigPeakOutputReverse(-1.0, ShooterConstants::kTimeoutMs);

  m_pivot.ConfigForwardSoftLimitEnable(true);
  m_pivot.ConfigForwardSoftLimitThreshold(ShooterConstants::kMinAimSensor - 50);
  m_pivot.ConfigReverseSoftLimitEnable(true);
  m_pivot.ConfigReverseSoftLimitThreshold(ShooterConstants::kMaxAimSensor +
                                          300);

  m_pivot.Config_kF(ShooterConstants::kPIDLoopIdx, ShooterConstants::kFPivot,
                    ShooterConstants::kTimeoutMs);
  m_pivot.Config_kP(ShooterConstants::kPIDLoopIdx, ShooterConstants::kPPivot,
                    ShooterConstants::kTimeoutMs);
  m_pivot.Config_kI(ShooterConstants::kPIDLoopIdx, ShooterConstants::kIPivot,
                    ShooterConstants::kTimeoutMs);
  m_pivot.Config_kD(ShooterConstants::kPIDLoopIdx, ShooterConstants::kDPivot,
                    ShooterConstants::kTimeoutMs);
  // Motors following + leading

  m_pivot.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::MotionMagic, 0);

  // set Motion Magic settings
  m_pivot.ConfigMotionCruiseVelocity(
      480); // 80 rps = 16384 ticks/100ms cruise velocity
  m_pivot.ConfigMotionAcceleration(
      1280); // 160 rps/s = 32768 ticks/100ms/s acceleration
  m_pivot.ConfigMotionSCurveStrength(0); // s-curve smoothing strength of 3

  // periodic, run Motion Magic with slot 0 configs
  m_pivot.SelectProfileSlot(0, 0);

  m_leadMotor.SetInverted(true);

  m_followMotor.Follow(m_leadMotor, true);

  m_followMotor.SetInverted(false);

  m_leadMotor.SetOpenLoopRampRate(0.1);
  m_followMotor.SetOpenLoopRampRate(0.1);

  m_leadMotor.SetClosedLoopRampRate(0.1);
  m_followMotor.SetClosedLoopRampRate(0.1);

  // m_leadMotor.SetSmartCurrentLimit(5);
  // m_followMotor.SetSmartCurrentLimit(5);

  m_goal = 0_deg;

  // m_map.insert(3.408470_m, 21.534357_deg);
  // m_map.insert(3.949039_m, 12.174549_deg);
  m_map.insert(3.231028_m, 20.455068_deg);
  m_map.insert(1.424465_m, 41.296232_deg);
  m_map.insert(2.388467_m, 27.834950_deg);
  m_map.insert(2.438238_m, 25.355256_deg);
  m_map.insert(3.141301_m, 23.401145_deg);
  m_map.insert(2.640507_m, 25.355256_deg);
  m_map.insert(3.047757_m, 23.894969_deg);
  m_map.insert(4.079345_m, 14.305907_deg);
  m_map.insert(4.397344_m, 16.375658_deg);
  m_map.insert(3.872459_m, 18.299923_deg);
  m_map.insert(7.050149_m, 10_deg);

  frc::DataLogManager::Log(
      fmt::format("Finished initializing shooter subsystem."));
}

Shooter::~Shooter() {}

void Shooter::InitVisualization(frc::Mechanism2d *mech) {
  auto root = mech->GetRoot("shooter", 0.3048, 0.3048);

  m_mech_pivot_goal = root->Append<frc::MechanismLigament2d>(
      "aim goal",                 // name
      0.3048,                     // meters long
      180_deg,                    // start angle
      4,                          // pixel width
      frc::Color8Bit{20, 200, 20} // RGB, green
  );

  m_mech_mm_setpoint = root->Append<frc::MechanismLigament2d>(
      "aim motion magic", 0.3048, 180_deg, 1, frc::Color8Bit{80, 80, 200}
      // blueish
  );

  m_mech_pivot = root->Append<frc::MechanismLigament2d>(
      "aim", 0.3048, 180_deg, 10, frc::Color8Bit{240, 240, 240} // white
  );
}

void Shooter::UpdateVisualization() {
  const auto sensor_goal = m_pivot.GetClosedLoopTarget();
  const auto angle_goal = (sensor_goal - ShooterConstants::kMinAimSensor) /
                          ShooterConstants::kAngleToSensor;
  m_mech_pivot_goal->SetAngle(180_deg - angle_goal);

  // shows current status of motion magic trajectory
  const auto mm_sensor_setpoint = m_pivot.GetActiveTrajectoryPosition();
  const auto mm_angle_setpoint =
      (mm_sensor_setpoint - ShooterConstants::kMinAimSensor) /
      ShooterConstants::kAngleToSensor;
  m_mech_mm_setpoint->SetAngle(180_deg - mm_angle_setpoint);

  const auto sensor_measured = m_pivot.GetSelectedSensorPosition();
  const auto angle_measured =
      (sensor_measured - ShooterConstants::kMinAimSensor) /
      ShooterConstants::kAngleToSensor;
  m_mech_pivot->SetAngle(180_deg - angle_measured);

  // scale blueness of shooter on flywheel speed
  const auto wheel_vel = m_leadMotor.GetAppliedOutput();
  m_mech_pivot->SetColor(
      {240 - int(wheel_vel * 200), 240 - int(wheel_vel * 200), 240});
}

// Runs both shooting motors
void Shooter::RunShootMotor() {

  // Starts
  m_leadMotor.SetVoltage(1.0_V);
}

// Stops both shooting motors
void Shooter::StopShootMotor() {

  // Stops
  m_leadMotor.StopMotor();
}

// Runs pivoting motor
void Shooter::RunTalonMotor() {
  // Runs
  m_pivot.SetVoltage(1.0_V);
}

float pow(float d, int power) {
  float temp = d;
  for (int i = 0; i < power - 1; i++) {
    temp = temp * d;
  }
  return temp;
}

// Stop pivoting motor
void Shooter::StopTalonMotor() {
  // Stops
  m_pivot.StopMotor();
}

void Shooter::SetPivotMotor(double encoderPosition) {
  double clamped =
      std::clamp<double>(encoderPosition, ShooterConstants::kMaxIdeal,
                         ShooterConstants::kMinIdeal);

  m_pivot.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::MotionMagic,
              clamped);
}

void Shooter::Periodic() {
  // SmartDashboard
  frc::SmartDashboard::PutNumber("Shooter/Flywheel power", m_leadMotor.Get());
  frc::SmartDashboard::PutNumber("Shooter/Pivot power", m_pivot.Get());
  frc::SmartDashboard::PutNumber("Shooter/Pivot encoder",
                                 m_pivot.GetSelectedSensorPosition());

  UpdateVisualization();
}

units::degree_t Shooter::DistanceToAngle(units::foot_t distance) {
  return (std::atan(5.55 / distance.value()) - .15) * 1_rad;
}

units::degree_t Shooter::DistanceToAngleError(units::foot_t distance,
                                              units::radian_t angle) {
  return units::degree_t{
      distance.value() * (units::math::tan(angle).value()) -
      ((16.1 * std::pow(distance.value() - 0.77 * std::cos(angle.value()), 2)) /
       (std::pow((ShooterConstants::kNoteVelocity / 1_fps).value(), 2) *
        std::pow(std::cos(angle.value()), 2))) -
      5.55};
}
units::degree_t Shooter::DistanceToAngleBinarySearch(units::foot_t distance) {

  auto min = ShooterConstants::kMinAngle;
  auto max = ShooterConstants::kMaxAngle;
  auto pivot = max + min / 2;
  auto x = DistanceToAngleError(distance, pivot);

  for (int i = 0; i <= 10; i++) {

    if (x.value() < 0) {
      min = pivot;
    } else {
      max = pivot;
    }
    pivot = max + min / 2;

    x = DistanceToAngleError(distance, pivot);
  }

  return pivot;
}

double Shooter::distance_adjustment(units::feet_per_second_t robot_velocity,
                                    units::foot_t distance,
                                    units::radian_t thetaf) {
  constexpr units::feet_per_second_t note_velocity =
      ShooterConstants::kNoteVelocity; // not correct, also move to constant

  auto t = (robot_velocity.value() +
            (note_velocity.value() * std::cos(thetaf.value()))) /
           (distance.value() - 0.77 * (std::cos(thetaf.value())));

  return note_velocity.value() * t * std::sin(thetaf.value()) -
         ((0.5) * (32.15) * (t * t)) - 5.55 - 0.77 * (std::sin(thetaf.value()));
}

units::foot_t Shooter::DistanceAdjustmentBinarySearch(
    units::feet_per_second_t robot_velocity, units::foot_t distance,
    units::feet_per_second_t perp_velocity) {
  auto min = ShooterConstants::kMinAngle;
  auto max = ShooterConstants::kMaxAngle;
  units::radian_t pivot = max + min / 2;
  auto x = distance_adjustment(robot_velocity, distance, pivot);

  for (int i = 0; i <= 10; i++) {

    if (x < 0) {
      min = pivot;
    } else {
      max = pivot;
    }
    pivot = max + min / 2;

    x = distance_adjustment(robot_velocity, distance, pivot);
  }
  return 0_m;

  double t = (robot_velocity.value() +
              ((ShooterConstants::kNoteVelocity / 1_fps).value() *
               std::cos(pivot.value()))) /
             (distance.value() - 0.77 * (std::cos(pivot.value())));

  units::foot_t df =
      units::foot_t{robot_velocity.value() * t * std::cos(pivot.value())};

  // Should be using perpendicular velocity to complement zTargeting.
  auto dAdjust =
      df / std::cos(std::asin(
               (perp_velocity / ShooterConstants::kNoteVelocity).value()));

  return units::foot_t{robot_velocity.value() * t * std::cos(pivot.value())};
}

double Shooter::ToTalonUnits(const frc::Rotation2d &rotation) {
  units::radian_t currentHeading =
      m_pivot.GetSelectedSensorPosition() / ShooterConstants::kAngleToSensor;
  // Puts the rotation in the correct scope for the incremental encoder.
  return ((frc::AngleModulus(rotation.Radians() - currentHeading) +
           currentHeading) *
          ShooterConstants::kAngleToSensor)
             .value() +
         ShooterConstants::kMinAimSensor;
}

units::radian_t Shooter::GetAnglePivot() {
  return (units::radian_t{m_pivot.GetSelectedSensorPosition() *
                          ShooterConstants::kPivotEncoderDistancePerCount} +
          ShooterConstants::kOffset);
}

frc2::CommandPtr Shooter::ShooterVelocityCommand(
    std::function<double()> flywheelInput,
    std::function<units::angular_velocity::degrees_per_second_t()>
        pivotVelocity) {

  auto pivotAngle = [this, pivotVelocity] {
    m_goal += pivotVelocity() * 20_ms;

    if (m_goal < ShooterConstants::kMinAngle) {
      m_goal = ShooterConstants::kMinAngle;
    } else if (m_goal > ShooterConstants::kMaxAngle) {
      m_goal = ShooterConstants::kMaxAngle;
    };

    return m_goal;
  };

  return frc2::cmd::Parallel(FlywheelCommand(flywheelInput),
                             PivotAngleCommand(pivotAngle));
}

frc2::CommandPtr
Shooter::ShooterCommand(std::function<double()> flywheelInput,
                        std::function<units::meter_t()> calculateDistance) {
  return frc2::cmd::Parallel(FlywheelCommand(flywheelInput),
                             PivotAngleDistanceCommand(calculateDistance));
}

frc2::CommandPtr
Shooter::FlywheelCommand(std::function<double()> controllerInput) {
  return frc2::cmd::Run(
      [this, controllerInput] {
        m_leadMotor.SetVoltage(12_V * (controllerInput() * controllerInput()) /
                               2.0);
      },
      {});
}

frc2::CommandPtr
Shooter::PivotAngleCommand(std::function<units::degree_t()> angle) {
  return frc2::cmd::Run(
      [this, angle]() {
        SetPivotMotor(ToTalonUnits(angle()));
        frc::SmartDashboard::PutNumber("Shooter/Pivot Angle Goal",
                                       angle().value());
        frc::SmartDashboard::PutNumber("Shooter/Pivot Encoder Goal",
                                       ToTalonUnits(angle()));
      },
      {this});
}

frc2::CommandPtr
Shooter::PivotAngleDistanceCommand(std::function<units::meter_t()> distance) {
  return frc2::cmd::Run(
      [this, distance]() {
        SetPivotMotor(ToTalonUnits(m_map[distance()]));
        frc::SmartDashboard::PutNumber("Shooter/Pivot Input Dist.",
                                       distance().value());
        frc::SmartDashboard::PutNumber("Shooter/Pivot Angle Goal",
                                       DistanceToAngle(distance()).value());
        frc::SmartDashboard::PutNumber(
            "Shooter/Pivot Encoder Goal",
            ToTalonUnits(DistanceToAngle(distance())));
      },
      {this});
}

frc2::CommandPtr Shooter::PivotAngleVelocityDistanceCommand(
    std::function<units::foot_t()> distance,
    std::function<units::feet_per_second_t()> fwd_velocity,
    std::function<units::feet_per_second_t()> strafe_velocity) {
  return frc2::cmd::Run(
      [this, distance, fwd_velocity, strafe_velocity]() {
        // auto angle =
        // DistanceToAngleBinarySearch(DistanceAdjustmentBinarySearch(
        //         fwd_velocity(), distance(), strafe_velocity()));
        auto angle = DistanceToAngleBinarySearch(distance());
        SetPivotMotor(ToTalonUnits(angle));
        frc::SmartDashboard::PutNumber("Shooter/Pivot Input Dist.",
                                       distance().value());
        frc::SmartDashboard::PutNumber("Shooter/Pivot Angle Goal",
                                       angle.value());
        frc::SmartDashboard::PutNumber("Shooter/Pivot Encoder Goal",
                                       ToTalonUnits(angle));
      },
      {this});
}

// ************************ SIMULATION *****************************
void Shooter::SimulationPeriodic() {
  using namespace ShooterConstants;
  if (!m_sim_state)
    return;

  // Simulate main shooter motor
  units::volt_t applied_voltage{
      m_sim_state->m_leadShooterSim.GetDouble("Applied Output").Get()};

  m_sim_state->m_shooterModel.SetInputVoltage(applied_voltage);
  m_sim_state->m_shooterModel.Update(20_ms);
  m_sim_state->m_leadShooterSim.GetDouble("Velocity")
      .Set(m_sim_state->m_shooterModel.GetAngularVelocity()
               .convert<units::revolutions_per_minute>()
               .value());
  m_sim_state->m_followerShooterSim.GetDouble("Velocity")
      .Set(m_sim_state->m_shooterModel.GetAngularVelocity()
               .convert<units::revolutions_per_minute>()
               .value());
  m_sim_state->m_leadShooterSim.GetDouble("Motor Current")
      .Set(m_sim_state->m_shooterModel.GetCurrentDraw().value());

  // Simulate arm
  m_sim_state->m_aimMotorSim.SetBusVoltage(
      frc::RobotController::GetBatteryVoltage().value());

  m_sim_state->m_armModel.SetInputVoltage(
      -units::volt_t{m_sim_state->m_aimMotorSim.GetMotorOutputLeadVoltage()});
  m_sim_state->m_armModel.Update(20_ms);

  const units::degree_t arm_angle{m_sim_state->m_armModel.GetAngle()};
  const auto sensor_angle_reading =
      kMinAimSensor + kAngleToSensor * (arm_angle - kMinAngle);
  m_sim_state->m_aimMotorSim.SetAnalogPosition(sensor_angle_reading);

  // Talon expects speed in terms of "sensor units per 100ms"
  // hence multiplying by 100_ms
  const units::degrees_per_second_t arm_speed{
      m_sim_state->m_armModel.GetVelocity()};
  const auto sensor_speed_reading = arm_speed * kAngleToSensor * 100_ms;
  m_sim_state->m_aimMotorSim.SetAnalogVelocity(sensor_speed_reading);

  m_sim_state->m_aimMotorSim.SetSupplyCurrent(
      m_sim_state->m_armModel.GetCurrentDraw().value());

  frc::SmartDashboard::PutNumber("Shooter/Analog Position",
                                 sensor_angle_reading.value());
  frc::SmartDashboard::PutNumber("Shooter/Analog Velocity",
                                 sensor_speed_reading.value());
}
