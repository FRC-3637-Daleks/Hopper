#pragma once

#include <optional>

#include <frc2/command/SubsystemBase.h>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/PhotonTargetSortMode.h>
#include <photon/PhotonUtils.h>
#include <photon/targeting/PhotonPipelineResult.h>
#include <photon/targeting/PhotonTrackedTarget.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <Eigen/Core>
#include <math.h>
#include <wpi/array.h>

#include <memory>

namespace VisionConstants {

constexpr std::string_view kPhotonShooterCameraName =
    "ArduCam_OV2310_Usb_Camera"; // Note, we need an in-built pipeline changer,
                                 // probably between auton and teleop
constexpr std::string_view kPhotonIntakeCameraName =
    "Arducam_OV9281_USB_Camera";

const frc::Transform3d kShooterCameraToRobot{
    {-4_in, -5.5_in, 23_in},
    frc::Rotation3d{// transform3d can be constructed with a variety of
                    // variables, so this should be fine
                    90_deg, 0_deg,
                    180_deg}}; // The camera location relative to the robot's
                               // center. Need to change for actual robot

/**A Transform3d that defines the Intake camera offset from the zero (center of
 * robot, between all 4 swerve modules)*/

const frc::Transform3d kIntakeCameraToRobot{
    {-4_in, 0_in, 23_in},
    frc::Rotation3d{// transform3d can be constructed with a variety of
                    // variables, so this should be fine
                    180_deg, 0_deg, 0_deg}};

inline const frc::AprilTagFieldLayout kTagLayout{
    frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo)};
inline const Eigen::Matrix<double, 3, 1> kSingleTagStdDevs{0.2, 0.2, 1};
inline const Eigen::Matrix<double, 3, 1> kMultiTagStdDevs{0.1, 0.1, 0.5};
inline const Eigen::Matrix<double, 3, 1> kFailedTagStdDevs{
    std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
    std::numeric_limits<double>::max()};
} // namespace VisionConstants

class VisionSim; // forward declaration

class Vision : public frc2::SubsystemBase {

public:
  Vision(
      std::function<void(frc::Pose2d, units::second_t, wpi::array<double, 3U>)>
          addVisionMeasurement,
      std::function<frc::Pose2d()> getRobotPose,
      const Eigen::Matrix<double, 3, 1> &initialStdDevs,
      std::function<frc::Pose2d()> getSimulatedPose);
  ~Vision();

  // photon::PhotonPoseEstimator m_estimator;

  void Periodic() override;
  void SimulationPeriodic() override;

  void GetBestPose();

  bool HasTargets();

  std::optional<photon::EstimatedRobotPose>
  CalculateRobotPoseEstimate(photon::PhotonPoseEstimator &estimator);
  /**Gets the standard deviation of the pose returned by
   * CalculateRobotPoseEstimate*/
  Eigen::Matrix<double, 3, 1>
  GetEstimationStdDevs(frc::Pose2d estimatedPose,
                       photon::PhotonPoseEstimator &estimator);
  // ...
public:
  bool IsPoseWithinStdDevs(const frc::Pose2d &incomingPose);
  // ...

private:
  photon::PhotonCamera m_shooterCamera{
      VisionConstants::kPhotonShooterCameraName};
  photon::PhotonCamera m_intakeCamera{VisionConstants::kPhotonIntakeCameraName};
  photon::PhotonPoseEstimator m_shooterEstimator;
  photon::PhotonPoseEstimator m_intakeEstimator;

  std::optional<photon::EstimatedRobotPose> m_intakeApriltagEstimate{
      std::nullopt};
  std::optional<photon::EstimatedRobotPose> m_shooterApriltagEstimate{
      std::nullopt};
  // explicit PhotonPoseEstimator(frc::AprilTagFieldLayout aprilTags,
  //                          PoseStrategy strategy, PhotonCamera&& camera,
  //                          frc::Transform3d robotToCamera);
  Eigen::Matrix<double, 3, 1> m_estimatedStdDevs;
  units::time::second_t lastEstTimestamp;
  std::function<void(frc::Pose2d, units::second_t, wpi::array<double, 3U>)>
      m_addVisionMeasurement;

  std::function<frc::Pose2d()> m_referencePose;

private:
  friend class VisionSim;
  std::unique_ptr<VisionSim> m_sim_state;
};