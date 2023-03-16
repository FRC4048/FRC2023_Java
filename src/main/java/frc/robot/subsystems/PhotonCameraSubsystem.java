package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.apriltags.AprilTagMap;
import frc.robot.utils.SmartShuffleboard;

public class PhotonCameraSubsystem extends SubsystemBase {

  private PhotonCamera camera;
  private AprilTagFieldLayout layout;
  private Pose2d robotFieldPose;
  PhotonPoseEstimator estimator;
  private Pose3d tagFieldPosition;
  private int noTagDetectedCounter;
  // private boolean isRedAlliance;
  private Alliance currentAlliance;
  private double timestamp;
  private EstimatedRobotPose estimatedPose;
  int targetId;

  private DataLog dataLog = new DataLog();
  private DoubleLogEntry dataLogEntryPose2dX = new DoubleLogEntry(dataLog, "/photonLog/Pose2d_X");
  private DoubleLogEntry dataLogEntryPose2dY = new DoubleLogEntry(dataLog, "/photonLog/Pose2d_Y");
  private DoubleLogEntry dataLogEntryPose2dRotation = new DoubleLogEntry(dataLog, "/photonLog/Pose2dRotationRadians");
  private IntegerLogEntry dataLogEntryTagDetected = new IntegerLogEntry(dataLog, "/photonLog/tagDetected");

  // TODO Adjust constant based on actual camera to robot height
  // TODO: Add constant to shift to center of robot (or wherever needed)
  Transform3d camToRobot = new Transform3d(
      new Translation3d(0.0, 0, -.47),
      new Rotation3d(0, 0, 0));

  public PhotonCameraSubsystem() {
    camera = new PhotonCamera("camera0");
    camera.setDriverMode(false);
    currentAlliance = DriverStation.getAlliance();

    layout = AprilTagMap.getAprilTagLayout(currentAlliance);
    estimator = new PhotonPoseEstimator(layout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, camToRobot);

  }

  private void updateAlliance() {
    if (currentAlliance != DriverStation.getAlliance()) {
      currentAlliance = DriverStation.getAlliance();

      layout = AprilTagMap.getAprilTagLayout(currentAlliance);
      estimator = new PhotonPoseEstimator(layout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, camToRobot);

      SmartShuffleboard.put("AprilTag", "currentAlliance", currentAlliance == Alliance.Red);
    }

  }

  private void calculateUsingEstimator() {
    if (camera.isConnected()) {
      Optional<EstimatedRobotPose> result = estimator.update();

      if (result.isPresent()) {
        estimatedPose = result.get();
        targetId = estimatedPose.targetsUsed.get(0).getFiducialId();
        tagFieldPosition = layout.getTagPose(targetId).get();
        robotFieldPose = estimatedPose.estimatedPose.toPose2d();
        noTagDetectedCounter = 0;
      } else {
        if (robotFieldPose != null) {
          noTagDetectedCounter++;
          if (noTagDetectedCounter >= 10) {
            robotFieldPose = null;
            noTagDetectedCounter = 0;
            estimatedPose = null;
            targetId = 0;
            tagFieldPosition = null;
          }
        }

      }
    }
  }

  public double getDetectionTimestamp() {
    if (estimatedPose == null) {
      return -1;
    } else {
      timestamp = estimatedPose.timestampSeconds;
      return timestamp;
    }

  }

  /**
   * Gets the latest robotFieldPose, could be null
   * 
   * @return
   */

  public Pose2d getRobot2dFieldPose() {
    return robotFieldPose;

  }

  @Override
  public void periodic() {
    updateAlliance();
    calculateUsingEstimator();
    Pose3d pose3dPosition = null;
    if (estimatedPose != null) {
      pose3dPosition = estimatedPose.estimatedPose;
    }

    if (Constants.APRILTAG_LOGGING) {
      if (getRobot2dFieldPose() != null) {
        dataLogEntryPose2dX.append(getRobot2dFieldPose().getX());
        dataLogEntryPose2dY.append(getRobot2dFieldPose().getY());
        dataLogEntryPose2dRotation.append(getRobot2dFieldPose().getRotation().getRadians());
      }
      else {
        dataLogEntryPose2dX.append(-111111);
        dataLogEntryPose2dY.append(-111111);
        dataLogEntryPose2dRotation.append(-111111); 
      }
      dataLogEntryTagDetected.append(targetId);

    }

    if (Constants.APRILTAG_DEBUG) {
      SmartShuffleboard.put("AprilTag", "isConnected", camera.isConnected());
      if (robotFieldPose != null) {
        SmartShuffleboard.put("AprilTag", "2D", "2D-X", robotFieldPose.getTranslation().getX());
        SmartShuffleboard.put("AprilTag", "2D", "2D-Y", robotFieldPose.getTranslation().getY());
        SmartShuffleboard.put("AprilTag", "2D", "Angle", robotFieldPose.getRotation().getDegrees());
        SmartShuffleboard.put("AprilTag", "2D", "AngleR", robotFieldPose.getRotation().getRadians());

      } else {
        SmartShuffleboard.put("AprilTag", "2D", "2D-X", 0);
        SmartShuffleboard.put("AprilTag", "2D", "2D-Y", 0);
        SmartShuffleboard.put("AprilTag", "2D", "Angle", 0);
        SmartShuffleboard.put("AprilTag", "2D", "AngleR", 0);

      }
      SmartShuffleboard.put("AprilTag", "noTagDetectedCounter", noTagDetectedCounter);
      if (pose3dPosition != null) {
        SmartShuffleboard.put("AprilTag", "3D", "3D-X", pose3dPosition.getX());
        SmartShuffleboard.put("AprilTag", "3D", "3D-Y", pose3dPosition.getY());
        SmartShuffleboard.put("AprilTag", "3D", "3D-Z", pose3dPosition.getZ());
        SmartShuffleboard.put("AprilTag", "3D", "3D-RX", pose3dPosition.getRotation().getX());
        SmartShuffleboard.put("AprilTag", "3D", "3D-RY", pose3dPosition.getRotation().getY());
        SmartShuffleboard.put("AprilTag", "3D", "3D-RZ", pose3dPosition.getRotation().getZ());
      }

      if (tagFieldPosition != null) {
        SmartShuffleboard.put("AprilTag", "position-x", tagFieldPosition.getX());
        SmartShuffleboard.put("AprilTag", "position-y", tagFieldPosition.getY());
      } else {
        SmartShuffleboard.put("AprilTag", "position-x", 0);
        SmartShuffleboard.put("AprilTag", "position-y", 0);
      }
    }
  }
}
