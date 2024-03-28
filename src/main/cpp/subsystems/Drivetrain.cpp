#include "subsystems/Drivetrain.h"

#include <cmath> // Make sure to include cmath for std::fmod

#include <frc/DataLogManager.h>
#include <frc/I2C.h>
#include <frc/SPI.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/simulation/LinearSystemSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/ProfiledPIDCommand.h>
#include <frc2/command/WaitCommand.h>
#include <wpi/array.h>

#include <hal/SimDevice.h>
#include <hal/simulation/SimDeviceData.h>

using namespace DriveConstants;

class DrivetrainSimulation {
public:
  DrivetrainSimulation(Drivetrain &drivetrain)
      : m_gyroYaw(HALSIM_GetSimValueHandle(
            HALSIM_GetSimDeviceHandle("navX-Sensor[4]"), "Yaw")),
        m_poseSim(drivetrain.kDriveKinematics, drivetrain.GetHeading(),
                  {drivetrain.m_frontLeft.GetPosition(),
                   drivetrain.m_frontRight.GetPosition(),
                   drivetrain.m_rearLeft.GetPosition(),
                   drivetrain.m_rearRight.GetPosition()}) {}

public:
  hal::SimDouble m_gyroYaw;
  frc::SwerveDriveOdometry<4> m_poseSim;
};

Drivetrain::Drivetrain()
    : m_frontLeft{"FL",
                  kFrontLeftDriveMotorId,
                  kFrontLeftSteerMotorId,
                  kFrontLeftAbsoluteEncoderChannel,
                  kFrontLeftDriveMotorPIDCoefficients,
                  kFrontLeftSteerMotorPIDCoefficients},
      m_rearLeft{"RL",
                 kRearLeftDriveMotorId,
                 kRearLeftSteerMotorId,
                 kRearLeftAbsoluteEncoderChannel,
                 kRearLeftDriveMotorPIDCoefficients,
                 kRearLeftSteerMotorPIDCoefficients},
      m_frontRight{"FR",
                   kFrontRightDriveMotorId,
                   kFrontRightSteerMotorId,
                   kFrontRightAbsoluteEncoderChannel,
                   kFrontRightDriveMotorPIDCoefficients,
                   kFrontRightSteerMotorPIDCoefficients},
      m_rearRight{"RR",
                  kRearRightDriveMotorId,
                  kRearRightSteerMotorId,
                  kRearRightAbsoluteEncoderChannel,
                  kRearRightDriveMotorPIDCoefficients,
                  kRearRightSteerMotorPIDCoefficients},
      m_gyro{frc::SPI::Port::kMXP},
      m_poseEstimator{kDriveKinematics, GetGyroHeading(),
                      wpi::array<frc::SwerveModulePosition, 4U>{
                          m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                          m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                      frc::Pose2d()},
      m_sim_state(new DrivetrainSimulation(*this)) {
  frc::DataLogManager::Log(
      fmt::format("Finished initializing drivetrain subsystem."));
}
frc::Pose2d Drivetrain::GetSimulatedGroundTruth() {
  return m_sim_state->m_poseSim.GetPose();
}
void Drivetrain::Periodic() {

  // Do this once per loop
  SwerveModule::RefreshAllSignals(m_frontLeft, m_frontRight, m_rearLeft,
                                  m_rearRight);

  // const auto fr_pos = m_frontRight.GetPosition();
  // const auto rl_pos = m_rearLeft.GetPosition();
  // const auto rr_pos = m_rearRight.GetPosition();
  // const auto fl_pos = m_frontLeft.GetPosition();

  // const auto fr_pos = m_frontRight.GetPosition();
  // const auto rl_pos = m_rearLeft.GetPosition();
  // const auto fl_pos = m_frontLeft.GetPosition();
  // const auto rr_pos = m_rearRight.GetPosition();

  // const auto fr_pos = m_frontRight.GetPosition();
  // const auto fl_pos = m_frontLeft.GetPosition();
  // const auto rl_pos = m_rearLeft.GetPosition();
  // const auto rr_pos = m_rearRight.GetPosition();

  // Update the odometry with the current gyro angle and module states.
  auto fl_pos = m_frontLeft.GetPosition();
  auto fr_pos = m_frontRight.GetPosition();
  auto rl_pos = m_rearLeft.GetPosition();
  auto rr_pos = m_rearRight.GetPosition();

  auto prev_pose = m_poseEstimator.GetEstimatedPosition();
  m_poseEstimator.Update(GetGyroHeading(), {fl_pos, fr_pos, rl_pos, rr_pos});
  auto new_pose = m_poseEstimator.GetEstimatedPosition();

  auto rel_transform = new_pose - prev_pose;
  auto dist = rel_transform.Translation().Norm();

  auto corrected_pose = new_pose.TransformBy(
      {0_m, -dist * DriveConstants::kOdometryCompensationFactor, 0_deg});

  // Forgive me God for I have sinned
  // -- Eric
  m_poseEstimator.AddVisionMeasurement(
      corrected_pose, wpi::math::MathSharedStore::GetTimestamp(),
      {0.0, 0.0, 0.0});

  this->UpdateDashboard();
}

Drivetrain::~Drivetrain() {}

void Drivetrain::Drive(units::meters_per_second_t forwardSpeed,
                       units::meters_per_second_t strafeSpeed,
                       units::radians_per_second_t angularSpeed,
                       bool fieldRelative, bool isRed) {
  //  Use the kinematics model to get from the set of commanded speeds to a set
  //  of states that can be commanded to each module.
  auto states = kDriveKinematics.ToSwerveModuleStates(
      frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});

  if (fieldRelative) {
    if (isRed)
      states = kDriveKinematics.ToSwerveModuleStates(
          frc::ChassisSpeeds::FromFieldRelativeSpeeds(
              -1 * forwardSpeed, -1 * strafeSpeed, angularSpeed, GetHeading()));
    else
      states = kDriveKinematics.ToSwerveModuleStates(
          frc::ChassisSpeeds::FromFieldRelativeSpeeds(
              forwardSpeed, strafeSpeed, angularSpeed, GetHeading()));
  } else {
    states = kDriveKinematics.ToSwerveModuleStates(
        frc::ChassisSpeeds{forwardSpeed, strafeSpeed, angularSpeed});
  }

  // Occasionally a drive motor is commanded to go faster than its maximum
  // output can sustain. Desaturation lowers the module speeds so that no motor
  // is driven above its maximum speed, while preserving the intended motion.
  kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  // Finally each of the desired states can be sent as commands to the modules.
  auto [fl, fr, rl, rr] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(rl);
  m_rearRight.SetDesiredState(rr);
}

void Drivetrain::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         DriveConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

frc::Rotation2d Drivetrain::GetHeading() { return GetPose().Rotation(); }

frc::Rotation2d Drivetrain::GetGyroHeading() {
  return units::degree_t(-m_gyro.GetYaw());
}

void Drivetrain::ZeroHeading() { m_gyro.Reset(); }

void Drivetrain::ZeroAbsEncoders() {
  m_frontLeft.ZeroAbsEncoders();
  m_frontRight.ZeroAbsEncoders();
  m_rearLeft.ZeroAbsEncoders();
  m_rearRight.ZeroAbsEncoders();
}

void Drivetrain::SetAbsEncoderOffset() {
  m_frontLeft.SetEncoderOffset();
  m_frontRight.SetEncoderOffset();
  m_rearLeft.SetEncoderOffset();
  m_rearRight.SetEncoderOffset();
}

void Drivetrain::SyncEncoders() {
  m_frontLeft.SyncEncoders();
  m_frontRight.SyncEncoders();
  m_rearLeft.SyncEncoders();
  m_rearRight.SyncEncoders();
}

void Drivetrain::CoastMode(bool coast) {
  m_frontLeft.CoastMode(coast);
  m_frontRight.CoastMode(coast);
  m_rearLeft.CoastMode(coast);
  m_rearRight.CoastMode(coast);
}
units::degrees_per_second_t Drivetrain::GetTurnRate() {
  return -m_gyro.GetRate() * 1_deg_per_s;
}

frc::Pose2d Drivetrain::GetPose() {
  auto translation = m_poseEstimator.GetEstimatedPosition();
  auto new_translation = (translation + m_odometryCompensation).Translation();
  return frc::Pose2d{new_translation,
                     m_poseEstimator.GetEstimatedPosition().Rotation()};
}

frc::ChassisSpeeds Drivetrain::GetSpeed() {
  return kDriveKinematics.ToChassisSpeeds(
      m_frontLeft.GetState(), m_frontRight.GetState(), m_rearLeft.GetState(),
      m_rearRight.GetState());
}

void Drivetrain::ResetOdometry(const frc::Pose2d &pose) {
  m_poseEstimator.ResetPosition(
      GetGyroHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}

void Drivetrain::UpdateDashboard() {
  const auto robot_center = this->GetPose();
  m_field.SetRobotPose(this->GetPose());

  const auto fl_pose = robot_center.TransformBy(
      {kWheelBase / 2, kTrackWidth / 2, m_frontLeft.GetState().angle});
  m_field.GetObject("FL")->SetPose(fl_pose);

  const auto fr_pose = robot_center.TransformBy(
      {kWheelBase / 2, -kTrackWidth / 2, m_frontRight.GetState().angle});
  m_field.GetObject("FR")->SetPose(fr_pose);

  const auto rl_pose = robot_center.TransformBy(
      {-kWheelBase / 2, kTrackWidth / 2, m_rearLeft.GetState().angle});
  m_field.GetObject("RL")->SetPose(rl_pose);

  const auto rr_pose = robot_center.TransformBy(
      {-kWheelBase / 2, -kTrackWidth / 2, m_rearRight.GetState().angle});
  m_field.GetObject("RR")->SetPose(rr_pose);

  frc::SmartDashboard::PutData("Field", &m_field);

  frc::SmartDashboard::PutBoolean("Swerve/Gyro calibrating?",
                                  m_gyro.IsCalibrating());
  frc::SmartDashboard::PutNumber("Swerve/Robot heading",
                                 GetHeading().Degrees().value());
  double swerveStates[] = {m_frontLeft.GetState().angle.Radians().value(),
                           m_frontLeft.GetState().speed.value(),
                           m_frontRight.GetState().angle.Radians().value(),
                           m_frontRight.GetState().speed.value(),
                           m_rearLeft.GetState().angle.Radians().value(),
                           m_rearLeft.GetState().speed.value(),
                           m_rearRight.GetState().angle.Radians().value(),
                           m_rearRight.GetState().speed.value()};
  frc::SmartDashboard::PutNumberArray(
      "Swerve/Swerve Module States",
      swerveStates); // Have to initialize array separately due as an error
                     // occurs when an array attempts to initialize as a
                     // parameter.

  frc::SmartDashboard::PutData("zeroEncodersCommand",
                               zeroEncodersCommand.get());
  m_frontLeft.UpdateDashboard();
  m_rearLeft.UpdateDashboard();
  m_frontRight.UpdateDashboard();
  m_rearRight.UpdateDashboard();

  frc::SmartDashboard::PutNumber("Swerve/Gyro", m_gyro.GetAngle());

  frc::SmartDashboard::PutData("PDH", &m_pdh);

  frc::SmartDashboard::PutData("Swerve/TurnPIDController", &m_turnPID);

  // frc::SmartDashboard::PutNumber("PDH/Voltage", m_pdh.GetVoltage());

  // frc::SmartDashboard::PutNumber("PDH/Total Current",
  // m_pdh.GetTotalCurrent());
}

void Drivetrain::SimulationPeriodic() {
  if (!m_sim_state)
    return;

  m_frontLeft.SimulationPeriodic();
  m_rearLeft.SimulationPeriodic();
  m_frontRight.SimulationPeriodic();
  m_rearRight.SimulationPeriodic();

  // Assume perfect kinematics and get the new gyro angle
  const auto chassis_speed = kDriveKinematics.ToChassisSpeeds(
      m_frontLeft.GetState(), m_frontRight.GetState(), m_rearLeft.GetState(),
      m_rearRight.GetState());

  const auto theta = m_sim_state->m_poseSim.GetPose().Rotation();
  const auto new_theta =
      theta.RotateBy(units::radian_t{chassis_speed.omega * 20_ms});
  // robot nav x defines clockwise as positive instead of counterclockwise
  m_sim_state->m_gyroYaw.Set(-new_theta.Degrees().value());

  // Feed this simulated gyro angle into the odometry to get simulated position
  auto fl_pos = m_frontLeft.GetPosition();
  auto fr_pos = m_frontRight.GetPosition();
  auto rl_pos = m_rearLeft.GetPosition();
  auto rr_pos = m_rearRight.GetPosition();

  // Modify this to simulate different kinds of odom error
  fl_pos.angle = fl_pos.angle.Degrees() * 1.02;
  fr_pos.angle = fr_pos.angle.Degrees() * 0.99;
  rl_pos.angle = rl_pos.angle.Degrees();
  rr_pos.angle = rr_pos.angle.Degrees() * 1.01;

  m_sim_state->m_poseSim.Update(new_theta, {fl_pos, fr_pos, rl_pos, rr_pos});

  m_field.GetObject("simulation")->SetPose(m_sim_state->m_poseSim.GetPose());
}

frc2::CommandPtr Drivetrain::SwerveCommand(
    std::function<units::meters_per_second_t()> forward,
    std::function<units::meters_per_second_t()> strafe,
    std::function<units::revolutions_per_minute_t()> rot) {
  // fmt::print("making command\n");
  return this->Run([=] {
    // fmt::print("starting drive command\n");
    Drive(forward(), strafe(), rot(), false, false);
    // fmt::print("sent drive command\n");
  });
}

frc2::CommandPtr Drivetrain::SwerveCommandFieldRelative(
    std::function<units::meters_per_second_t()> forward,
    std::function<units::meters_per_second_t()> strafe,
    std::function<units::revolutions_per_minute_t()> rot,
    std::function<bool()> isRed) {
  return this->Run([=] { Drive(forward(), strafe(), rot(), true, isRed()); });
}

frc2::CommandPtr Drivetrain::SwerveSlowCommand(
    std::function<units::meters_per_second_t()> forward,
    std::function<units::meters_per_second_t()> strafe,
    std::function<units::revolutions_per_minute_t()> rot,
    std::function<bool()> isRed) {
  // fmt::print("making command\n");
  return this->Run([=] {
    // fmt::print("starting drive command\n");
    Drive(forward() / 4, strafe() / 4, rot() / 5, true, isRed());
    // fmt::print("sent drive command\n");
  });
}

frc2::CommandPtr Drivetrain::ZeroHeadingCommand() {
  return this->RunOnce([&] { ZeroHeading(); });
}

frc2::CommandPtr Drivetrain::ZeroAbsEncodersCommand() {
  return this
      ->RunOnce([&] {
        fmt::print("inside ZeroAbsEncodersCommand");
        ZeroAbsEncoders();
      })
      .IgnoringDisable(true);
};

frc2::CommandPtr Drivetrain::SetAbsEncoderOffsetCommand() {
  return this->RunOnce([&] { SetAbsEncoderOffset(); }).IgnoringDisable(true);
}

frc2::CommandPtr Drivetrain::CoastModeCommand(bool coast) {
  return this->StartEnd([&] { this->CoastMode(true); },
                        [&] { this->CoastMode(false); });
}

frc2::CommandPtr Drivetrain::ConfigAbsEncoderCommand() {
  return this
      ->StartEnd(
          [&] {
            fmt::print("insdie the configabscommand ********* ");
            CoastMode(true);
            ZeroAbsEncoders();
          },
          [&] {
            CoastMode(false);
            SetAbsEncoderOffset();
          })
      .AndThen(frc2::WaitCommand(0.5_s).ToPtr())
      .AndThen(this->RunOnce([&] { SyncEncoders(); }))
      .IgnoringDisable(true);
}

void Drivetrain::AddVisionPoseEstimate(
    frc::Pose2d pose, units::second_t timestamp,
    wpi::array<double, 3U> visionMeasurementStdDevs) {
  m_poseEstimator.AddVisionMeasurement(pose, timestamp,
                                       visionMeasurementStdDevs);

  m_field.GetObject("vision estimate")->SetPose(pose);
}

frc2::CommandPtr Drivetrain::TurnToAngleCommand(units::degree_t angle) {
  /*
      Profiled PID Command for turning to angle.
      m_turnPID - Profiled PID Controller, in degrees.
      Lambda to measure angle.
      Final state: angle at 0 radians per second.
      Move at setpoint velocity from Profiled PID Controller.
  */
  // continuous and wraps around
  m_turnPID.EnableContinuousInput(-180_deg, 180_deg);

  return frc2::ProfiledPIDCommand<units::degree>(
             m_turnPID,
             [this]() -> units::degree_t {
               auto currentAngle = GetHeading().Degrees();
               frc::SmartDashboard::PutNumber(
                   "TurnPID/Current Angle",
                   currentAngle.value()); // Debugging print
               // return GetHeading().Degrees();
               return currentAngle;
             },
             {angle, 0_rad_per_s},
             [this](double output,
                    frc::TrapezoidProfile<units::degree>::State setpoint) {
               Drive(0_mps, 0_mps,
                     setpoint.velocity +
                         units::angular_velocity::radians_per_second_t(output),
                     false, false);
               // Debugging print
               frc::SmartDashboard::PutNumber("TurnPID/PID Output", output);
               frc::SmartDashboard::PutNumber(
                   "TurnPID/Setpoint Velocity",
                   setpoint.velocity.value()); // Debugging print
               frc::SmartDashboard::PutNumber(
                   "TurnPID/Setpoint Position",
                   setpoint.position.value()); // Debugging print
               double pidVal[] = {DriveConstants::kPTurn,
                                  DriveConstants::kITurn,
                                  DriveConstants::kDTurn};
               frc::SmartDashboard::PutNumberArray("TurnPID/PID Val", pidVal);
             },
             {this})
      .ToPtr();
}

frc2::CommandPtr Drivetrain::ZTargetPoseCommand(
    std::function<frc::Pose2d()> pose,
    std::function<units::meters_per_second_t()> forward,
    std::function<units::meters_per_second_t()> strafe, bool shooterSide,
    std::function<bool()> isRed) {

  auto angle = [this, pose, shooterSide, strafe]() -> units::radian_t {
    auto rawAngle = units::math::atan2<units::meter_t, units::meter_t>(
        pose().Y() - GetPose().Y(), pose().X() - GetPose().X());
    return shooterSide
               ? (rawAngle + std::numbers::pi * 1_rad +
                  units::math::asin(strafe() / DriveConstants::kNoteVelocity))
               : rawAngle;
  };

  return frc2::ProfiledPIDCommand<units::degree>(
             m_turnPID,
             [this, angle]() -> units::degree_t {
               auto currentAngle =
                   frc::AngleModulus(GetPose().Rotation().Degrees() - angle());
               frc::SmartDashboard::PutNumber(
                   "TurnPID/Current Angle",
                   currentAngle.value()); // Debugging print
               // return GetHeading().Degrees();
               return currentAngle;
             },
             {0_rad, 0_rad_per_s},
             [this, forward, strafe,
              isRed](double output,
                     frc::TrapezoidProfile<units::degree>::State setpoint) {
               Drive(forward(), strafe(),
                     setpoint.velocity +
                         units::angular_velocity::radians_per_second_t(output),
                     true, isRed());
               // Debugging print
               frc::SmartDashboard::PutNumber("TurnPID/PID Output", output);
               frc::SmartDashboard::PutNumber(
                   "TurnPID/Setpoint Velocity",
                   setpoint.velocity.value()); // Debugging print
               frc::SmartDashboard::PutNumber(
                   "TurnPID/Setpoint Position",
                   setpoint.position.value()); // Debugging print
               double pidVal[] = {DriveConstants::kPTurn,
                                  DriveConstants::kITurn,
                                  DriveConstants::kDTurn};
               frc::SmartDashboard::PutNumberArray("TurnPID/PID Val", pidVal);
             },
             {this})
      .ToPtr();
}