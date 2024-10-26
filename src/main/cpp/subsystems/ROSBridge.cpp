#include "subsystems/ROSBridge.h"

#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <frc/DriverStation.h>

ROSBridge::ROSBridge() {
  m_ntInst = nt::NetworkTableInstance::NetworkTableInstance::GetDefault();

  m_ntInst.StartClient4("RosDrivetrain");

  m_pubOdomTimestamp =
      m_ntInst.GetIntegerTopic("/Drivetrain/nt2ros/odom/timestamp").Publish();
  m_pubOdomPosLinear =
      m_ntInst.GetDoubleArrayTopic("/Drivetrain/nt2ros/odom/position/linear")
          .Publish();
  m_pubOdomPosAngular =
      m_ntInst.GetDoubleArrayTopic("/Drivetrain/nt2ros/odom/position/angular")
          .Publish();
  m_pubOdomVelLinear =
      m_ntInst.GetDoubleArrayTopic("/Drivetrain/nt2ros/odom/velocity/linear")
          .Publish();
  m_pubOdomVelAngular =
      m_ntInst.GetDoubleArrayTopic("/Drivetrain/nt2ros/odom/velocity/angular")
          .Publish();
  m_pubOdomAccLinear =
      m_ntInst
          .GetDoubleArrayTopic("/Drivetrain/nt2ros/odom/acceleration/linear")
          .Publish();

  m_fmsTable = m_ntInst.GetTable("FMSInfo");
}

void ROSBridge::CheckFMS() {
  using DS = frc::DriverStation;
  m_fmsTable->PutString("EventName", DS::GetEventName());
  m_fmsTable->PutString("GameSpecificMessage", DS::GetGameSpecificMessage());
  m_fmsTable->PutNumber("StationNumber", DS::GetLocation().value_or(0));
  m_fmsTable->PutNumber("MatchType", DS::GetMatchType());
  m_fmsTable->PutNumber("MatchNumber", DS::GetMatchNumber());
  m_fmsTable->PutNumber("ReplayNumber", DS::GetReplayNumber());
  m_fmsTable->PutBoolean("IsRedAlliance",
                         DS::GetAlliance() == DS::Alliance::kRed);
}

void ROSBridge::PubOdom(const frc::Pose2d &pose,
                        const frc::ChassisSpeeds &vel) {
  auto current_time = nt::Now();
  m_pubOdomTimestamp.Set(current_time, current_time);

  double pubPosLinear[3] = {units::meter_t{pose.X()}.value(),
                            units::meter_t{pose.Y()}.value(), 0};
  m_pubOdomPosLinear.Set(pubPosLinear, current_time);

  double pubPosAngular[3] = {0, 0, pose.Rotation().Radians().value()};
  m_pubOdomPosAngular.Set(pubPosAngular, current_time);

  double pubVelLinear[3] = {units::meters_per_second_t{vel.vx}.value(),
                            units::meters_per_second_t{vel.vy}.value(), 0};
  m_pubOdomVelLinear.Set(pubVelLinear, current_time);

  double pubVelAngular[3] = {0, 0,
                             units::radians_per_second_t{vel.omega}.value()};
  m_pubOdomVelAngular.Set(pubVelAngular, current_time);

  m_ntInst.Flush();
}
