#include "subsystems/Vision.h"
#include "subsystems/Drivetrain.h"

#include <frc/DataLogManager.h>

// top do --> add the code from photonvision example to both the cpp and h files
// copy std deviation formula, and potentially make a system where the code
// resets to a set pose if no fiducials are found
//   or I can have it reset to odometry if no fiducials are found
// make code for robot to decide based off of if fidicual id's fall into
// standard deviation, which also means making standard deviation stricter than
// that in code
// Take in code as a Pose3d and convert to Pose2d for drivetrain and other
// estimation, as robot is only moving in 2d, (climb doesn't count") Fix the
// code so it understand difference between old pose and new pose dependent on
// the timestamp

Vision::Vision(
    std::function<void(frc::Pose2d, units::second_t, wpi::array<double, 3U>)>
        addVisionMeasurement,
    std::function<frc::Pose2d()> getRobotPose,
    const Eigen::Matrix<double, 3, 1> &initialStdDevs)
    : m_estimator(
          frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo),
          photon::MULTI_TAG_PNP_ON_COPROCESSOR,
          std::move(m_camera), // change to the multitag detection algorithm
          VisionConstants::kCameraToRobot),
      m_referencePose(getRobotPose) {
  // Inside the constructor body, you can perform additional operations if
  // needed
  m_addVisionMeasurement =
      addVisionMeasurement; // Call the addVisionMeasurement function
  m_estimator.SetMultiTagFallbackStrategy(
      photon::PoseStrategy::CLOSEST_TO_LAST_POSE);

  frc::DataLogManager::Log(
      fmt::format("Finished initializing vision subsystem."));
}

bool Vision::HasTargets() {
  photon::PhotonPipelineResult result = m_camera.GetLatestResult();

  return result.HasTargets();
}

std::optional<photon::EstimatedRobotPose> Vision::CalculateRobotPoseEstimate() {
  m_estimator.SetReferencePose(frc::Pose3d{m_referencePose()});
  auto visionEst = m_estimator.Update();
  auto camera = m_estimator.GetCamera();
  units::second_t latestTimestamp = camera->GetLatestResult().GetTimestamp();
  bool newResult =
      units::math::abs(latestTimestamp - lastEstTimestamp) > 1e-5_s;
  if (newResult) {
    lastEstTimestamp = latestTimestamp;
  }
  return visionEst;
}

Eigen::Matrix<double, 3, 1>
Vision::GetEstimationStdDevs(frc::Pose2d estimatedPose) {

  Eigen::Matrix<double, 3, 1> estStdDevs = VisionConstants::kSingleTagStdDevs;
  photon::PhotonPipelineResult latestResult =
      m_estimator.GetCamera()
          ->GetLatestResult(); // Add declaration for GetLatestResult function

  int numTags = 0; // Declare the variable "numTags" and initialize it to 0

  auto targets = latestResult.GetTargets();
  auto avgDist = 0.0_m; // Declare and initialize the variable "avgDist"

  for (const auto &tgt : targets) {
    auto tagPose = m_estimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
    if (tagPose.has_value()) {
      numTags++;
      avgDist += tagPose.value().ToPose2d().Translation().Distance(
          estimatedPose.Translation());
    }
  }
  if (numTags == 0) {
    fmt::println("0 tags!?! avg dist {}", avgDist);
    return VisionConstants::kFailedTagStdDevs;
  }
  avgDist /= numTags;
  if (numTags > 1) {
    estStdDevs = VisionConstants::kMultiTagStdDevs;
  }
  if (numTags == 1 && avgDist > 1_m || avgDist > 4_m) {
    estStdDevs =
        (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
         std::numeric_limits<double>::max(), std::numeric_limits<double>::max())
            .finished();
  } else {
    estStdDevs = estStdDevs * (1 + (avgDist.value() * avgDist.value() / 5));
  }

  frc::SmartDashboard::PutNumber("Vision/average vision distance",
                                 avgDist.value());
  return estStdDevs;
}

void Vision::Periodic() {
  auto PoseEst = CalculateRobotPoseEstimate();
  if (PoseEst.has_value()) {
    auto EstPose2d = PoseEst.value().estimatedPose.ToPose2d();
    auto StdDev = GetEstimationStdDevs(EstPose2d);
    wpi::array<double, 3U> StdDevArray{StdDev[0], StdDev[1], StdDev[2]};
    m_addVisionMeasurement(EstPose2d, lastEstTimestamp, StdDevArray);
  }
}
