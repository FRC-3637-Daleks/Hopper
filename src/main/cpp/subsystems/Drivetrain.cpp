#include "subsystems/Drivetrain.h"

#include <frc/I2C.h>
#include <frc/SPI.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/array.h>
#include <frc/simulation/LinearSystemSim.h>

using namespace DriveConstants;

class DrivetrainSimulation
{
public:
  DrivetrainSimulation(Drivetrain& drivetrain):
    m_gyroYaw(HALSIM_GetSimValueHandle(HALSIM_GetSimDeviceHandle("navX-Sensor[4]"), "Yaw")),
    m_poseSim(
      drivetrain.kDriveKinematics,
      drivetrain.GetHeading(),
      {drivetrain.m_frontLeft.GetPosition(),
       drivetrain.m_frontRight.GetPosition(),
       drivetrain.m_rearLeft.GetPosition(),
       drivetrain.m_rearRight.GetPosition()})
  {}

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
      m_poseEstimator{kDriveKinematics, GetHeading(),
                      wpi::array<frc::SwerveModulePosition, 4U>{
                          m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                          m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                      frc::Pose2d()},
      m_sim_state(new DrivetrainSimulation(*this)) { }

void Drivetrain::Periodic() {
  // Update the odometry with the current gyro angle and module states.
  m_poseEstimator.Update(
      GetHeading(), {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                     m_rearLeft.GetPosition(), m_rearRight.GetPosition()});
  m_field.SetRobotPose(m_poseEstimator.GetEstimatedPosition());

  this->UpdateDashboard();
}

Drivetrain::~Drivetrain() {}

void Drivetrain::Drive(units::meters_per_second_t forwardSpeed,
                       units::meters_per_second_t strafeSpeed,
                       units::radians_per_second_t angularSpeed,
                       bool fieldRelative) {
  //  Use the kinematics model to get from the set of commanded speeds to a set
  //  of states that can be commanded to each module.
  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative
          ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                forwardSpeed, strafeSpeed, angularSpeed, GetHeading())
          : frc::ChassisSpeeds{forwardSpeed, strafeSpeed, angularSpeed});

  // Occasionally a drive motor is commanded to go faster than its maximum
  // output can sustain. Desaturation lowers the module speeds so that no motor
  // is driven above its maximum speed, while preserving the intended motion.
  kDriveKinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  // fmt::print("desaturated wheel speeds\n");

  // Finally each of the desired states can be sent as commands to the modules.
  auto [fl, fr, rl, rr] = states;

  // fmt::print("setting swerve module states\n");
  // fmt::print("{} FL\n", fl.speed.value());
  // fmt::print("{} FR\n", fr.speed.value());
  // fmt::print("{} RL\n", rl.speed.value());
  // fmt::print("{} RR\n", rr.speed.value());

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

frc::Rotation2d Drivetrain::GetHeading() { return units::degree_t(-m_gyro.GetYaw()); }

void Drivetrain::ZeroHeading() { m_gyro.Reset(); }

units::degrees_per_second_t Drivetrain::GetTurnRate() {
  return -m_gyro.GetRate() * 1_deg_per_s;
}

frc::Pose2d Drivetrain::GetPose() {
  return m_poseEstimator.GetEstimatedPosition();
}

void Drivetrain::ResetOdometry(const frc::Pose2d &pose) {
  m_poseEstimator.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}

void Drivetrain::UpdateDashboard() {
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
  m_frontLeft.UpdateDashboard();
  m_rearLeft.UpdateDashboard();
  m_frontRight.UpdateDashboard();
  m_rearRight.UpdateDashboard();

  frc::SmartDashboard::PutNumber("Swerve/Gyro", m_gyro.GetAngle());

  frc::SmartDashboard::PutNumber("PDH/Voltage", m_pdh.GetVoltage());

  frc::SmartDashboard::PutNumber("PDH/Total Current", m_pdh.GetTotalCurrent());
}

void Drivetrain::SimulationPeriodic()
{
  if (!m_sim_state) return;

  m_frontLeft.SimulationPeriodic();
  m_rearLeft.SimulationPeriodic();
  m_frontRight.SimulationPeriodic();
  m_rearRight.SimulationPeriodic();

  // Assume perfect kinematics and get the new gyro angle
  const auto chassis_speed = kDriveKinematics.ToChassisSpeeds(
    m_frontLeft.GetState(), m_frontRight.GetState(),
    m_rearLeft.GetState(), m_rearRight.GetState());
  
  const auto theta = m_sim_state->m_poseSim.GetPose().Rotation();
  const auto new_theta = theta.RotateBy(units::radian_t{chassis_speed.omega*20_ms});
  // robot nav x defines clockwise as positive instead of counterclockwise
  m_sim_state->m_gyroYaw.Set(-new_theta.Degrees().value());

  // Feed this simulated gyro angle into the odometry to get simulated position
  m_sim_state->m_poseSim.Update(new_theta,
    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
     m_rearLeft.GetPosition(), m_rearRight.GetPosition()});
  
  m_field.GetObject("simulation")->SetPose(m_sim_state->m_poseSim.GetPose());
}

frc2::CommandPtr Drivetrain::SwerveCommand(
    std::function<units::meters_per_second_t()> forward,
    std::function<units::meters_per_second_t()> strafe,
    std::function<units::revolutions_per_minute_t()> rot) {
  // fmt::print("making command\n");
  return this->Run([=] {
    // fmt::print("starting drive command\n");
    Drive(forward(), strafe(), rot(), false);
    // fmt::print("sent drive command\n");
  });
}

frc2::CommandPtr Drivetrain::SwerveCommandFieldRelative(
    std::function<units::meters_per_second_t()> forward,
    std::function<units::meters_per_second_t()> strafe,
    std::function<units::revolutions_per_minute_t()> rot) {
  return this->Run([=] { Drive(forward(), strafe(), rot(), true); });
}

// // needs work
// frc2::CommandPtr Drivetrain::DriveToPoseCommand(frc::Pose2d targetPose) {
//   // TODO
//   frc::Pose2d startPose = m_odometry.GetPose();
//   const double kDistanceTolerance =
//       0.1; // Tolerance for position error in meters

//   units::meter_t hypotenuse =
//       units::meter_t{std::hypot((targetPose.X() - startPose.X()).value(),
//                            (targetPose.Y() - startPose.Y()).value())};

//   while (std::hypot((targetPose.X() - startPose.X()).value(),
//                     (targetPose.Y() - startPose.Y()).value()) >
//          kDistanceTolerance) {
//     // todo
//     break;
//   }

//   return {nullptr};
// }

// // Check if the robot has reached the target pose
// // need to fix
// bool Drivetrain::IsFinished(frc::Pose2d targetPose) {

//   frc::Pose2d startPose = m_odometry.GetPose();

//   auto distanceError =
//       targetPose.Translation().Distance(startPose.Translation());
//   auto angleError =
//       targetPose.Rotation().Radians() - startPose.Rotation().Radians();
//   return distanceError < kDistanceTolerance &&
//          std::fabs((double)angleError) < 0.05;
// }

frc2::CommandPtr Drivetrain::ZeroHeadingCommand() {
  return this->RunOnce([&] { ZeroHeading(); });
}

void Drivetrain::AddVisionPoseEstimate(frc::Pose2d pose,
                                       units::second_t timestamp) {
  m_poseEstimator.AddVisionMeasurement(pose, timestamp);
}