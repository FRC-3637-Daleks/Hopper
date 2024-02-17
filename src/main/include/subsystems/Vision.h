#pragma once

#include <optional>

#include <frc2/command/SubsystemBase.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPoseEstimator.h> 
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>


namespace VisionConstants
{
  constexpr std::string_view kPhotonCameraName =
      "ov23113637"; // Note, we need an in-built pipeline changer, probably between auton and teleop

  const frc::Transform3d kCameraToRobot{
      {14_in, 16_in, 30_in},
      frc::Rotation3d{ // transform3d can be constructed with a variety of variables, so this should be fine 
          90_deg, 0_deg,
          0_deg}, frc}; // The camera location relative to the robot's center. Need to change for actual robot
  const frc::AprilTagFieldLayout kAprilTagFieldLayout
  {
    frc::AprilTagFieldLayout aprilTagFieldLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);
    const frc::AprilTagFieldLayout kAprilTagFieldLayout{std::vector<frc::AprilTag>{
                                                            {0, frc::Pose3d(units::meter_t(3), units::meter_t(3), units::meter_t(3), frc::Rotation3d())},
                                                            {1, frc::Pose3d(units::meter_t(5), units::meter_t(5), units::meter_t(5), frc::Rotation3d())}},
                                                            54_ft, 27_ft};
                                                            // why do we need the different tags named?
  }
}

  class Vision : public frc2::SubsystemBase
  {
  public:
    Vision(std::function<void(frc::Pose3d, units::second_t)> addVisionMeasurement,
           std::function<frc::Pose3d()> getRobotPose);
           

    void Periodic() override;

    void GetBestPose();

    bool HasTargets();

    void CalculateRobotPoseEstimate();

  private:
    photonlib::PhotonCamera m_camera{VisionConstants::kPhotonCameraName};

    photonlib::PhotonPoseEstimator m_estimator;

    std::optional<photonlib::EstimatedRobotPose> m_apriltagEstimate{std::nullopt};

    std::function<void(frc::Pose3d, units::second_t)> m_addVisionMeasurement;
    std::function<frc::Pose3d()> m_getRobotPose;
  };
  // This is to create standard deviations for the vision system, which is used to determine if a pose is acurate enough to be used
  Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d estimatedPose) {
    Eigen::Matrix<double, 3, 1> estStdDevs =
        constants::Vision::kSingleTagStdDevs;
    auto targets = GetLatestResult().GetTargets();
    int numTags = 0;
    units::meter_t avgDist = 0_m;
    for (const auto& tgt : targets) {
      auto tagPose =
          photonEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
      if (tagPose.has_value()) {
        numTags++;
        avgDist += tagPose.value().ToPose2d().Translation().Distance(
            estimatedPose.Translation());
      }
    }
    if (numTags == 0) {
      return estStdDevs;
    }
    avgDist /= numTags;
    if (numTags > 1) {
      estStdDevs = constants::Vision::kMultiTagStdDevs;
    }
    if (numTags == 1 && avgDist > 4_m) {
      estStdDevs = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max())
                       .finished();
    } else {
      estStdDevs = estStdDevs * (1 + (avgDist.value() * avgDist.value() / 30));
    }
    return estStdDevs;
  }
  
