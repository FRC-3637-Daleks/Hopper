#include "subsystems/Vision.h"
#include "subsystems/Drivetrain.h"


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
          photon::MULTI_TAG_PNP_ON_COPROCESSOR, std::move(m_camera), // change to the multitag detection algorithm
          VisionConstants::kCameraToRobot} {
          addVisionMeasurement;
          getRobotPose;
          }

bool Vision::HasTargets() {
  photon::PhotonPipelineResult result = m_camera.GetLatestResult();

  return result.HasTargets();
}
// auto addMeasurementLambda = [](Vision& vision, frc::Pose3d pose, units::second_t timestamp) {
//     vision.m_addVisionMeasurement(pose, timestamp);
// };
// bool Vision::IsPoseWithinStdDevs(const frc::Pose2d& incomingPose) {
//     // Calculate the standard deviations of the estimated pose
//     Eigen::Matrix<double, 3, 1> stdDevs = GetEstimationStdDevs(frc::Pose2d estimatedPose);

//     // Calculate the differences between the incoming pose and the estimated pose
//     double diffX = incomingPose.X().to<double>() - m_estimatedPose.X().to<double>();
//     double diffY = incomingPose.Y().to<double>() - m_estimatedPose.Y().to<double>();
//     double diffTheta = incomingPose.Rotation().Degrees().to<double>() - m_estimatedPose.Rotation().Degrees().to<double>();

//     // Check if the incoming pose falls within the standard deviations
//     return std::abs(diffX) <= stdDevs(0, 0) && std::abs(diffY) <= stdDevs(1, 0) && std::abs(diffTheta) <= stdDevs(2, 0);
// }

// This is to create standard deviations for the vision system, which is used to determine if a pose is acurate enough to be used
  Eigen::Matrix<double, 3, 1> Vision::GetEstimationStdDevs(frc::Pose2d estimatedPose) {
    

    Eigen::Matrix<double, 3, 1> estStdDevs =
      VisionConstants::kSingleTagStdDevs;
    photon::PhotonPipelineResult latestResult; // Add declaration for GetLatestResult function

    int numTags = 0; // Declare the variable "numTags" and initialize it to 0

    auto targets = latestResult.GetTargets();
    auto avgDist = 0.0_m; // Declare and initialize the variable "avgDist"

    for (const auto& tgt : targets) {
      auto tagPose =
      m_estimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
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
      estStdDevs = VisionConstants::kMultiTagStdDevs;
    }
    if (numTags == 1 && avgDist > 4_m) {
      estStdDevs = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max())
                       .finished();
    } else {
      estStdDevs = estStdDevs * (1 + (avgDist * avgDist / 30));
    }
    return estStdDevs;
  }

  