#pragma once

#include <optional>

#include <frc2/command/SubsystemBase.h>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/targeting/PhotonPipelineResult.h>
#include <photon/targeting/PhotonTrackedTarget.h>
#include <photon/PhotonTargetSortMode.h>
#include <photon/PhotonUtils.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <Eigen/Core> 
#include <math.h>


namespace VisionConstants{

  constexpr std::string_view kPhotonCameraName =
      "ov23113637"; // Note, we need an in-built pipeline changer, probably between auton and teleop

  const frc::Transform3d kCameraToRobot{
      {14_in, 16_in, 30_in},
      frc::Rotation3d{ // transform3d can be constructed with a variety of variables, so this should be fine 
          90_deg, 0_deg,
          0_deg} }; // The camera location relative to the robot's center. Need to change for actual robot
  inline const frc::AprilTagFieldLayout kTagLayout{
    frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo)};
  inline const Eigen::Matrix<double, 3, 1> kSingleTagStdDevs{4, 4, 8};
  inline const Eigen::Matrix<double, 3, 1> kMultiTagStdDevs{0.5, 0.5, 1};
} // namespace VisionConstants

  
  class Vision : public frc2::SubsystemBase{

  public:
    Vision(std::function<void(frc::Pose3d, units::second_t)> addVisionMeasurement,
           std::function<frc::Pose3d()> getRobotPose,
           const Eigen::Matrix<double, 3, 1>& initialStdDevs);
         
    // photon::PhotonPoseEstimator m_estimator;
           

    void Periodic() override;

    void GetBestPose();

    bool HasTargets();

    void CalculateRobotPoseEstimate();

    Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d estimatedPose);
    // ...
    public:
    bool IsPoseWithinStdDevs(const frc::Pose2d& incomingPose);
    // ...


  private:
    photon::PhotonCamera m_camera{VisionConstants::kPhotonCameraName};
    photon::PhotonPoseEstimator m_estimator;

    std::optional<photon::EstimatedRobotPose> m_apriltagEstimate{std::nullopt};
    // explicit PhotonPoseEstimator(frc::AprilTagFieldLayout aprilTags,
    //                          PoseStrategy strategy, PhotonCamera&& camera,
    //                          frc::Transform3d robotToCamera);
    Eigen::Matrix<double, 3, 1> m_estimatedStdDevs; 


};
  
