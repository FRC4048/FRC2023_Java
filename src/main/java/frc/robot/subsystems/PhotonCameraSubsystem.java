package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.apriltags.AprilTagMap;
import frc.robot.utils.SmartShuffleboard;

public class PhotonCameraSubsystem extends SubsystemBase {

  private static final String FMSINFO_TABLE = "FMSInfo";
  private static final String IS_RED_ALLIANCE = "IsRedAlliance";

  private PhotonCamera camera;
  private AprilTagFieldLayout layout;
  private Pose2d robotFieldPose;
  PhotonPoseEstimator estimator;
  private Pose3d tagFieldPosition;

  private boolean isRedAlliance;

  int targetId;

  Field2d field = new Field2d();

  // TODO Adjust constant based on actual camera to robot height
  // TODO: Add constant to shift to center of robot (or wherever needed)
  Transform3d camToRobot = new Transform3d(
    new Translation3d(0.0, 0, -.47),
    new Rotation3d(0, 0, 0));

  public PhotonCameraSubsystem() {
    camera = new PhotonCamera("camera0");
    camera.setDriverMode(false);
    try {
      NetworkTable fmsInfoTable = NetworkTableInstance.getDefault().getTable(FMSINFO_TABLE);
      if (fmsInfoTable != null) {
        NetworkTableEntry isRedAllianceEntry = fmsInfoTable.getEntry(IS_RED_ALLIANCE);
        if (isRedAllianceEntry != null) {
          isRedAlliance = isRedAllianceEntry.getBoolean(true);
          layout = AprilTagMap.getAprilTagLayout(isRedAlliance);
        }
      } else {
        layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      }
    } catch (Exception e) {
      e.printStackTrace();
    }

    estimator = new PhotonPoseEstimator(layout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, camToRobot);

    SmartDashboard.putData(field);
  }

  private void checkAlliance() {
    NetworkTable fmsInfoTable = NetworkTableInstance.getDefault().getTable(FMSINFO_TABLE);

    if (fmsInfoTable != null) {
      NetworkTableEntry isRedAllianceEntry = fmsInfoTable.getEntry(IS_RED_ALLIANCE);

      if (isRedAllianceEntry != null) {
        boolean isRed = isRedAllianceEntry.getBoolean(false);
        if (isRedAlliance != isRed) {
          isRedAlliance = isRed;
          layout = AprilTagMap.getAprilTagLayout(isRedAlliance);
          estimator = new PhotonPoseEstimator(layout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, camToRobot);
        }
      }  
    }
    SmartShuffleboard.put("AprilTag", "isRedAlliance", isRedAlliance);
  }

  private Pose3d calculateUsingEstimator() {
    Optional<EstimatedRobotPose> result = estimator.update();
    if (result.isPresent()) {
        EstimatedRobotPose estimatedPose = result.get();
        targetId = estimatedPose.targetsUsed.get(0).getFiducialId();
        tagFieldPosition = layout.getTagPose(targetId).get();
        Pose3d pose = estimatedPose.estimatedPose;
        return pose;
    }
    return null;
  }

  /**
   * Gets the latest robotFieldPose, could be null
   * @return
   */
  public Pose2d getRobot2dFieldPose() {
    return robotFieldPose;
  }

  public Pose3d getRobotFieldPose() {
    Pose3d result = calculateUsingEstimator();
    if (result != null) {
        robotFieldPose = result.toPose2d();
        return result;
    }
    return null;
  }

  @Override
  public void periodic() {
    checkAlliance();
    Pose3d pose3dPosition = getRobotFieldPose();
    if (robotFieldPose != null) {
      SmartShuffleboard.put("AprilTag", "2D", "2D-X", robotFieldPose.getTranslation().getX());
      SmartShuffleboard.put("AprilTag", "2D", "2D-Y", robotFieldPose.getTranslation().getY());
      SmartShuffleboard.put("AprilTag", "2D", "Angle", robotFieldPose.getRotation().getDegrees());
      SmartShuffleboard.put("AprilTag", "2D", "AngleR", robotFieldPose.getRotation().getRadians());

      field.setRobotPose(robotFieldPose);
    } else {
      SmartShuffleboard.put("AprilTag", "2D", "2D-X", 0);
      SmartShuffleboard.put("AprilTag", "2D", "2D-Y", 0);
      SmartShuffleboard.put("AprilTag", "2D", "Angle", 0);
      SmartShuffleboard.put("AprilTag", "2D", "AngleR", 0);

      field.setRobotPose(new Pose2d(-100, -100, new Rotation2d(0)));
    }

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
