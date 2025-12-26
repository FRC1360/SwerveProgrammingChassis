package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class SwerveCamera {
  private final AprilTagFieldLayout aprilTagFieldLayout;

  private final PhotonCamera photonCamera;
  private final PhotonPoseEstimator photonPoseEstimator;
  private final Transform3d robotToCamera;

  private List<PhotonPipelineResult> currentUnreadResults;

  StructPublisher<Pose2d> photonPose;
  private Matrix<N3, N1> curStdDevs;
  private Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 0);
  private Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  /* Simulation Stuff */
  private final PhotonCameraSim photonCameraSim;
  private final SimCameraProperties photonCameraSimProperties;
  private final VisionSystemSim photonVisionSystemSim;

  public SwerveCamera(Transform3d robotToCamera, String cameraName) {
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    this.robotToCamera = robotToCamera;
    this.photonCamera = new PhotonCamera(cameraName);
    photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.robotToCamera);

    photonPose =
        NetworkTableInstance.getDefault()
            .getStructTopic(cameraName + "/Photon_Pose", Pose2d.struct)
            .publish();

    this.currentUnreadResults = new ArrayList<PhotonPipelineResult>();

    /* Simulation Settings */
    photonVisionSystemSim = new VisionSystemSim(cameraName + "-main");
    photonVisionSystemSim.addAprilTags(aprilTagFieldLayout);

    photonCameraSimProperties = new SimCameraProperties();
    photonCameraSimProperties.setCalibration(640, 400, Rotation2d.fromDegrees(68));
    photonCameraSimProperties.setCalibError(0.25, 0.08);
    photonCameraSimProperties.setFPS(50);
    photonCameraSimProperties.setAvgLatencyMs(21);
    photonCameraSimProperties.setLatencyStdDevMs(3);

    photonCameraSim = new PhotonCameraSim(photonCamera, photonCameraSimProperties);

    photonVisionSystemSim.addCamera(photonCameraSim, robotToCamera);

    // Enable the raw and processed streams. These are enabled by default.
    photonCameraSim.enableRawStream(true);
    photonCameraSim.enableProcessedStream(true);

    // Enable drawing a wireframe visualization of the field to the camera streams.
    // This is extremely resource-intensive and is disabled by default.
    photonCameraSim.enableDrawWireframe(true);
  }

  public void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  public PhotonCamera getPhotonCamera() {
    return this.photonCamera;
  }

  public PhotonPoseEstimator getPhotonPoseEstimator() {
    return this.photonPoseEstimator;
  }

  public void updateStructPublisher(Pose2d cameraEstimatedPose) {
    photonPose.accept(cameraEstimatedPose);
  }

  // Functions for updating and retreiving pipeling results
  public void updatePipelineResults() {
    currentUnreadResults = this.photonCamera.getAllUnreadResults();
  }

  public List<PhotonPipelineResult> getPipelineResults() {
    return currentUnreadResults;
  }

  // Update Vision Pose
  public void updateSimPose(Pose2d robotPose) {
    photonVisionSystemSim.update(robotPose);
  }

  public Transform3d getRobotToCamera() {
    return this.robotToCamera;
  }
}
