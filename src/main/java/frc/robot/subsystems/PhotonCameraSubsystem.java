package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SmartShuffleboard;

public class PhotonCameraSubsystem extends SubsystemBase {

  private PhotonCamera camera;
  private AprilTagFieldLayout layout;
  private Pose2d builtIn2D;
  PhotonPoseEstimator estimator;
  Optional<Pose3d> targetTagPose;
  

  int targetId;

  Field2d field = new Field2d();

  // -0.523598776
  static final Transform3d robotToCam = new Transform3d(
      new Translation3d(0.0, 0, .4318),
      new Rotation3d(0, 0, 0));

  public PhotonCameraSubsystem() {
    camera = new PhotonCamera("camera0");
    camera.setDriverMode(false);
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (Exception e) {
      e.printStackTrace();
    }

    estimator = new PhotonPoseEstimator(layout, PoseStrategy.LOWEST_AMBIGUITY, camera, robotToCam);

    SmartDashboard.putData(field);
  }

  // protected Pose2d calculatePoseEstimation() {
  // Optional<EstimatedRobotPose> estimate = estimator.update();
  // if (estimate.isPresent()) {
  // EstimatedRobotPose pose = estimate.get();
  // Pose3d poseFromEstimate = pose.estimatedPose;

  // Pose2d robotPoseFrom3d = new Pose2d(
  // poseFromEstimate.getX(),
  // poseFromEstimate.getY(),
  // new Rotation2d(poseFromEstimate.getRotation().getZ()));

  // return robotPoseFrom3d;
  // } else {
  // System.out.println("No Tag Found");
  // }
  // return null;
  // }

  private void calculatePose2d() {
    PhotonPipelineResult result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();

    if (hasTargets) {
      PhotonTrackedTarget target = result.getBestTarget();

      targetId = target.getFiducialId();
      Transform3d targetLocation = target.getBestCameraToTarget();
      targetTagPose = layout.getTagPose(targetId);
      if (targetTagPose.isPresent()) {
        Pose3d robotPosition = PhotonUtils.estimateFieldToRobotAprilTag(
            targetLocation,
            targetTagPose.get(),
            robotToCam);

        builtIn2D = robotPosition.toPose2d();
      } else {
        builtIn2D = null;
      }
    } else {
      builtIn2D = null;
      targetId = 0;
    }
  }

  @Override
  public void periodic() {
    SmartShuffleboard.put("AprilTag", "TestMessage", true);
    calculatePose2d();
    if (builtIn2D != null) {

      SmartShuffleboard.put("AprilTag", "2D", "2D-X", builtIn2D.getTranslation().getX());
      SmartShuffleboard.put("AprilTag", "2D", "2D-Y", builtIn2D.getTranslation().getY());
      SmartShuffleboard.put("AprilTag", "2D", "Angle", builtIn2D.getRotation().getDegrees());
      SmartShuffleboard.put("AprilTag", "2D", "AngleR", builtIn2D.getRotation().getRadians());

      // SmartShuffleboard.put("AprilTag", "RP", "X", robotPosition.getX());
      // SmartShuffleboard.put("AprilTag", "RP", "Y", robotPosition.getY());
      // SmartShuffleboard.put("AprilTag", "RP", "Z", robotPosition.getZ());
      // SmartShuffleboard.put("AprilTag", "RP", "RX", robotPosition.getRotation().getX());
      // SmartShuffleboard.put("AprilTag", "RP", "RY", robotPosition.getRotation().getY());
      // SmartShuffleboard.put("AprilTag", "RP", "RZ", robotPosition.getRotation().getZ());


      SmartShuffleboard.put("AprilTag", "tagposition-x", targetTagPose.get().getX());
      SmartShuffleboard.put("AprilTag", "tagposition-y", targetTagPose.get().getY());

      // SmartShuffleboard.put("AprilTag", "tagId", targetId);
      // SmartShuffleboard.put("AprilTag", "x", targetLocation.getX());
      // SmartShuffleboard.put("AprilTag", "y", targetLocation.getY());
      // SmartShuffleboard.put("AprilTag", "z", targetLocation.getZ());

      field.setRobotPose(builtIn2D);

    } else {
      SmartShuffleboard.put("AprilTag", "2D", "2D-X", 0);
      SmartShuffleboard.put("AprilTag", "2D", "2D-Y", 0);
      SmartShuffleboard.put("AprilTag", "2D", "Angle", 0);
      SmartShuffleboard.put("AprilTag", "2D", "AngleR", 0);

      field.setRobotPose(new Pose2d(-100, -100, new Rotation2d(0)));
    }
  }

  public Pose2d getPose2d() {
    calculatePose2d();
    return builtIn2D;
  }
}
