#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>

class ROSBridge {
public:
  ROSBridge();

public:
  void PubOdom(const frc::Pose2d &pose, const frc::ChassisSpeeds &twist);
  void CheckFMS();

private:
  nt::NetworkTableInstance m_ntInst;

  nt::IntegerPublisher m_pubOdomTimestamp;
  nt::DoubleArrayPublisher m_pubOdomPosLinear;
  nt::DoubleArrayPublisher m_pubOdomPosAngular;
  nt::DoubleArrayPublisher m_pubOdomVelLinear;
  nt::DoubleArrayPublisher m_pubOdomVelAngular;
  nt::DoubleArrayPublisher m_pubOdomAccLinear;

  std::shared_ptr<nt::NetworkTable> m_fmsTable;
};