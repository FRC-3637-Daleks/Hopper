#include "subsystems/Vision.h"
#include "subsystems/Drivetrain.h"

#include <photon/simulation/VisionSystemSim.h>
#include <photon/simulation/SimCameraProperties.h>

#include <frc/RobotBase.h>

class VisionSim
{
public:
  VisionSim(Vision &vision, std::function<frc::Pose2d()> getSimulatedPose);

  std::function<frc::Pose2d()> m_simulatedPose;
  photon::VisionSystemSim m_vision_sim;
  photon::PhotonCameraSim m_shooter_cam_sim;
};

// top do --> add the code from photonvsion example to both the cpp and h files
  // copy std deviation formula, and potentially make a system where the code resets to a set pose if no fiducials are found
    //   or I can have it reset to odometry if no fiducials are found 
  // make code for robot to decide based off of if fidicual id's fall into standard deviation, which also means making standard deviation stricter than that in code
// Take in code as a Pose3d and convert to Pose2d for drivetrain and other estimation, as robot is only moving in 2d, (climb doesn't count")
// Fix the code so it understand difference between old pose and new pose dependent on the timestamp 

Vision::Vision(
    std::function<void(frc::Pose2d, units::second_t, wpi::array<double, 3U>)> addVisionMeasurement,
    std::function<frc::Pose2d()> getRobotPose,
    const Eigen::Matrix<double, 3, 1>& initialStdDevs,
    std::function<frc::Pose2d()> getSimulatedPose) 
    : m_estimator(
        frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo),
        photon::MULTI_TAG_PNP_ON_COPROCESSOR,
        std::move(m_camera),  // change to the multitag detection algorithm
        VisionConstants::kCameraToRobot),
      m_referencePose(getRobotPose)
{
    if constexpr (frc::RobotBase::IsSimulation()) {
      m_sim_state.reset(new VisionSim(*this, std::move(getSimulatedPose)));
    }
    // Inside the constructor body, you can perform additional operations if needed
    m_addVisionMeasurement = addVisionMeasurement; // Call the addVisionMeasurement function
    m_estimator.SetMultiTagFallbackStrategy(photon::PoseStrategy::CLOSEST_TO_REFERENCE_POSE);
}

Vision::~Vision() {}

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


// This is to create standard deviations for the vision system, which is used to determine if a pose is acurate enough to be used
  Eigen::Matrix<double, 3, 1> Vision::GetEstimationStdDevs(frc::Pose2d estimatedPose) {
    

    Eigen::Matrix<double, 3, 1> estStdDevs =
      VisionConstants::kSingleTagStdDevs;
    photon::PhotonPipelineResult latestResult = m_estimator.GetCamera()->GetLatestResult(); // Add declaration for GetLatestResult function

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
      return VisionConstants::kFailedTagStdDevs;
    }
    avgDist /= numTags;
    if (numTags > 1) {
      estStdDevs = VisionConstants::kMultiTagStdDevs;
    }
    if (avgDist > 4_m) {
      estStdDevs = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max())
                       .finished();
    } else {
      estStdDevs = estStdDevs * (1 + (avgDist.value() * avgDist.value() / 5));
    }

    frc::SmartDashboard::PutNumber("Vision/average vision distance", avgDist.value());
    frc::SmartDashboard::PutNumber("Vision/num tags", numTags);
    return estStdDevs;
  }

  void Vision::Periodic()
  {
    m_apriltagEstimate = CalculateRobotPoseEstimate();
    if(m_apriltagEstimate)
    {
      auto EstPose2d = m_apriltagEstimate.value().estimatedPose.ToPose2d();
      auto StdDev = GetEstimationStdDevs(EstPose2d);
      wpi::array<double, 3U> StdDevArray{StdDev[0], StdDev[1], StdDev[2]};
      m_addVisionMeasurement(EstPose2d, lastEstTimestamp, StdDevArray);
    
    }
  }

/*****************************SIMULATION*****************************/

photon::SimCameraProperties getShooterCameraProperties()
{
  photon::SimCameraProperties ret;
  ret.SetCalibration(1600, 1200, 95_deg);
  ret.SetCalibError(0.15, 0.04);
  ret.SetFPS(15_Hz);
  ret.SetAvgLatency(0.04_s);
  ret.SetLatencyStdDev(0.01_s);
  
  return ret;
}

VisionSim::VisionSim(Vision &vision, std::function<frc::Pose2d()> getSimulatedPose):
  m_simulatedPose(std::move(getSimulatedPose)),
  m_vision_sim("april_tag_sim"),
  m_shooter_cam_sim(vision.m_estimator.GetCamera().get(), getShooterCameraProperties())
{
  m_vision_sim.AddAprilTags(frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo));
  m_vision_sim.AddCamera(&m_shooter_cam_sim, VisionConstants::kCameraToRobot);
  m_shooter_cam_sim.EnableDrawWireframe(true);
  m_shooter_cam_sim.EnabledProcessedStream(true);
  m_shooter_cam_sim.EnableRawStream(true);
  m_shooter_cam_sim.SetMaxSightRange(6_m);

  frc::SmartDashboard::PutData("Vision/simulated apriltags", &m_vision_sim.GetDebugField());
}

void Vision::SimulationPeriodic()
{
  if (!m_sim_state) return;

  m_sim_state->m_vision_sim.Update(m_sim_state->m_simulatedPose());
  m_sim_state->m_vision_sim.GetDebugField().GetObject("fused pose")->SetPose(m_referencePose());
  
  if (m_apriltagEstimate)
  {
    auto robot_pose = m_apriltagEstimate.value().estimatedPose;
    m_sim_state->m_vision_sim.GetDebugField().GetObject("photon pose est")->SetPose(robot_pose.ToPose2d());
    
    std::vector<frc::Pose2d> reprojected_tags;
    for (const auto &tag : m_estimator.GetCamera()->GetLatestResult().GetTargets())
    {
      auto tag_pose = robot_pose.TransformBy(VisionConstants::kCameraToRobot).TransformBy(tag.GetBestCameraToTarget());
      reprojected_tags.push_back(tag_pose.ToPose2d());
    }

    m_sim_state->m_vision_sim.GetDebugField().GetObject("tag poses")->SetPoses(reprojected_tags);
  }
}