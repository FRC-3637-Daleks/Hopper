#include "subsystems/Vision.h"
#include "subsystems/Drivetrain.h"

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <photon/targeting/PhotonPipelineResult.h>
#include <photon/targeting/PhotonTrackedTarget.h>
#include <photon/PhotonTargetSortMode.h>
#include <photon/PhotonUtils.h>

// top do --> add the code from photonvsion example to both the cpp and h files
  // copy std deviation formula, and potentially make a system where the code resets to a set pose if no fiducials are found
    //   or I can have it reset to odometry if no fiducials are found 
  // make code for robot to decide based off of if fidicual id's fall into standard deviation, which also means making standard deviation stricter than that in code
// Take in code as a Pose3d and convert to Pose2d for drivetrain and other estimation, as robot is only moving in 2d, (climb doesn't count")
// Fix the code so it understand difference between old pose and new pose dependent on the timestamp 

Vision::Vision(
    std::function<void(frc::Pose3d, units::second_t)> addVisionMeasurement,
    std::function<frc::Pose3d()> getRobotPose)
    : m_estimator{
          frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo),
          photonlib::MULTI_TAG_PNP_ON_COPROCESSOR, std::move(m_camera), // change to the multitag detection algorithm
          VisionConstants::kCameraToRobot} {
  m_addVisionMeasurement = addVisionMeasurement;
  m_getRobotPose = getRobotPose;
          }

bool Vision::HasTargets() {
  photonlib::PhotonPipelineResult result = m_camera.GetLatestResult();

  return result.HasTargets();
}
void Vision::CalculateRobotPoseEstimate() {

    m_estimator.SetReferencePose(frc::Pose3d{m_getRobotPose()});
    m_apriltagEstimate = m_estimator.Update();

    if (m_apriltagEstimate.has_value()) {
      m_addVisionMeasurement(
          m_apriltagEstimate.value().estimatedPose.ToPose2d(),
          m_apriltagEstimate.value().timestamp);
          double estStdDevs = GetEstimationStdDevs(m_apriltagEstimate.value().estimatedPose.ToPose2d());
          

  };
}



