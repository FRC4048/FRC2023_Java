package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.apriltags.AprilTagMap;
import frc.robot.apriltags.PoseFilter;
import frc.robot.utils.Logger;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.DiagPhotonVision;

public class PhotonCameraSubsystem extends SubsystemBase {

  private static final String FMSINFO_TABLE = "FMSInfo";
  private static final String IS_RED_ALLIANCE = "IsRedAlliance";

  private PoseFilter x2DFilter = new PoseFilter(3, 1); //Placeholder value in meters
  private PoseFilter y2DFilter = new PoseFilter(3, 1); //Placeholder value in meters
  private PoseFilter angleFilter = new PoseFilter(3, 0.15708); //Placeholder value in radians
  private PoseFilter x3DFilter = new PoseFilter(3, 1); //Placeholder value in meters
  private PoseFilter y3DFilter = new PoseFilter(3, 1); //Placeholder value in meters
  private PoseFilter z3DFilter = new PoseFilter(3, 1); //Placeholder value in meters
  private PoseFilter rX3DFilter = new PoseFilter(3, 0.15708); //Placeholder value in radians
  private PoseFilter rY3DFilter = new PoseFilter(3, 0.15708); //Placeholder value in radians
  private PoseFilter rZ3DFilter = new PoseFilter(3, 0.15708); //Placeholder value in radians

  private boolean useFilters = true;

  private PhotonCamera camera;
  private AprilTagFieldLayout layout;
  private Pose2d robotFieldPose;
  private Pose2d filteredRobotFieldPose;
  PhotonPoseEstimator estimator;
  private Pose3d tagFieldPosition;
  private int noTagDetectedCounter;
  // private boolean isRedAlliance;
  private Alliance currentAlliance;
  private double timestamp;
  private EstimatedRobotPose estimatedPose;
  private int periodicCounter = 0;

  int targetId;

  private NetworkTableEntry cameraLatency;



  // TODO Adjust constant based on actual camera to robot height
  // TODO: Add constant to shift to center of robot (or wherever needed)
  Transform3d camToRobot = new Transform3d(
      new Translation3d(0.0, 0, -.47),
      new Rotation3d(0, 0, 0));

  public PhotonCameraSubsystem() {
    camera = new PhotonCamera(Constants.PHOTON_CAMERA_ID);
    camera.setDriverMode(false);
    currentAlliance = DriverStation.getAlliance();

    NetworkTable cameraTable = NetworkTableInstance.getDefault().getTable(Constants.PHOTON_VISION_ID).getSubTable(Constants.PHOTON_CAMERA_ID);
    cameraLatency = cameraTable.getEntry(Constants.PHOTON_LATENCY);

    layout = AprilTagMap.getAprilTagLayout(currentAlliance);
    estimator = new PhotonPoseEstimator(layout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, camToRobot);

    Robot.getDiagnostics().addDiagnosable(new DiagPhotonVision("PV","CanReadTags") {
      @Override
      protected int getTagId() {
        
        return getTargetId();
      }

      @Override
      protected double getTagTimestamp() {
        calculateUsingEstimator();
        return getDetectionTimestamp();
      }
    });

  }

  private void updateAlliance() {
    if (currentAlliance != DriverStation.getAlliance()) {
      currentAlliance = DriverStation.getAlliance();

      layout = AprilTagMap.getAprilTagLayout(currentAlliance);
      estimator = new PhotonPoseEstimator(layout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, camToRobot);

      SmartShuffleboard.put("AprilTag", "currentAlliance", currentAlliance == Alliance.Red);
    }
  }

/**
   * Return the camera latency from network tables, will return -1 if no value is available
   * @return
   */
  public double getCameraLatencyMs() {
    return cameraLatency.getDouble(-1.0);
  }
  private void calculateUsingEstimator() {
    if (camera.isConnected()) {Optional<EstimatedRobotPose> result = estimator.update();

      if (result.isPresent()) {
        estimatedPose = result.get();
        targetId = estimatedPose.targetsUsed.get(0).getFiducialId();
        tagFieldPosition = layout.getTagPose(targetId).get();
        robotFieldPose = estimatedPose.estimatedPose.toPose2d();
        noTagDetectedCounter = 0;
        filteredRobotFieldPose = new Pose2d(x2DFilter.calculate(robotFieldPose.getTranslation().getX()), y2DFilter.calculate(robotFieldPose.getTranslation().getY()), new Rotation2d(angleFilter.calculate(robotFieldPose.getRotation().getRadians())));
      } else {
        if (robotFieldPose != null) {
          noTagDetectedCounter++;
          if (noTagDetectedCounter >= 10) {
            robotFieldPose = null;
            noTagDetectedCounter = 0;
            estimatedPose = null;
            targetId = 0;
            tagFieldPosition = null;
            filteredRobotFieldPose = null;
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

  public Pose2d getFilteredRobot2dFieldPose() {
    if(useFilters) {
      return filteredRobotFieldPose;
    }
    return robotFieldPose;
  }

  @Override
  public void periodic() {

    if (periodicCounter % 6 == 0) {
      periodicCounter = 1;
      //continue periodic
    }
    else {
      periodicCounter++;
      return;  //break out
    }

    updateAlliance();
    calculateUsingEstimator();
    Pose3d pose3dPosition = null;
    if (estimatedPose != null) {
      pose3dPosition = estimatedPose.estimatedPose;
    }

    Logger.logPose2d("/odometry/vision", robotFieldPose, Constants.ENABLE_LOGGING);
    Logger.logInteger("/vision/tagID", targetId, Constants.ENABLE_LOGGING);

    if (Constants.APRILTAG_DEBUG) {
      SmartShuffleboard.put("AprilTag", "isConnected", camera.isConnected());
        if (useFilters) {
          if (filteredRobotFieldPose != null) {
          SmartShuffleboard.put("AprilTag", "2D", "2D-X", filteredRobotFieldPose.getTranslation().getX());
          SmartShuffleboard.put("AprilTag", "2D", "2D-Y", filteredRobotFieldPose.getTranslation().getY());
          SmartShuffleboard.put("AprilTag", "2D", "Angle", filteredRobotFieldPose.getRotation().getDegrees());
          SmartShuffleboard.put("AprilTag", "2D", "AngleR", filteredRobotFieldPose.getRotation().getRadians());
          }
          else {
            x2DFilter.resetFilter();
            y2DFilter.resetFilter();
            angleFilter.resetFilter();
            SmartShuffleboard.put("AprilTag", "2D", "2D-X", 0);
            SmartShuffleboard.put("AprilTag", "2D", "2D-Y", 0);
            SmartShuffleboard.put("AprilTag", "2D", "Angle", 0);
            SmartShuffleboard.put("AprilTag", "2D", "AngleR", 0);
          }
        }
        else {
          if (robotFieldPose != null) {
          SmartShuffleboard.put("AprilTag", "2D", "2D-X", robotFieldPose.getTranslation().getX());
          SmartShuffleboard.put("AprilTag", "2D", "2D-Y", robotFieldPose.getTranslation().getY());
          SmartShuffleboard.put("AprilTag", "2D", "Angle", robotFieldPose.getRotation().getDegrees());
          SmartShuffleboard.put("AprilTag", "2D", "AngleR", robotFieldPose.getRotation().getRadians());
          }
        else {
            SmartShuffleboard.put("AprilTag", "2D", "2D-X", 0);
            SmartShuffleboard.put("AprilTag", "2D", "2D-Y", 0);
            SmartShuffleboard.put("AprilTag", "2D", "Angle", 0);
            SmartShuffleboard.put("AprilTag", "2D", "AngleR", 0);
          }

      }
      SmartShuffleboard.put("AprilTag", "noTagDetectedCounter", noTagDetectedCounter);
      if (pose3dPosition != null) {
        if (useFilters) {
          SmartShuffleboard.put("AprilTag", "3D", "3D-X", x3DFilter.calculate(pose3dPosition.getX()));
          SmartShuffleboard.put("AprilTag", "3D", "3D-Y", y3DFilter.calculate(pose3dPosition.getY()));
          SmartShuffleboard.put("AprilTag", "3D", "3D-Z", z3DFilter.calculate(pose3dPosition.getZ()));
          SmartShuffleboard.put("AprilTag", "3D", "3D-RX", rX3DFilter.calculate(pose3dPosition.getRotation().getX())); 
          SmartShuffleboard.put("AprilTag", "3D", "3D-RY", rY3DFilter.calculate(pose3dPosition.getRotation().getY())); 
          SmartShuffleboard.put("AprilTag", "3D", "3D-RZ", rZ3DFilter.calculate(pose3dPosition.getRotation().getZ()));
        }
        else {
          SmartShuffleboard.put("AprilTag", "3D", "3D-X", pose3dPosition.getX());
          SmartShuffleboard.put("AprilTag", "3D", "3D-Y", pose3dPosition.getY());
          SmartShuffleboard.put("AprilTag", "3D", "3D-Z", pose3dPosition.getZ());
          SmartShuffleboard.put("AprilTag", "3D", "3D-RX", pose3dPosition.getRotation().getX()); 
          SmartShuffleboard.put("AprilTag", "3D", "3D-RY", pose3dPosition.getRotation().getY()); 
          SmartShuffleboard.put("AprilTag", "3D", "3D-RZ", pose3dPosition.getRotation().getZ());
        }   
      }
      else {
        x3DFilter.resetFilter();
        y3DFilter.resetFilter();
        z3DFilter.resetFilter();
        rX3DFilter.resetFilter();
        rY3DFilter.resetFilter();
        rZ3DFilter.resetFilter();
      }

      if (tagFieldPosition != null) {
        SmartShuffleboard.put("AprilTag", "position-x", tagFieldPosition.getX());
        SmartShuffleboard.put("AprilTag", "position-y", tagFieldPosition.getY());
      } 
      else {
          SmartShuffleboard.put("AprilTag", "position-x", 0);
          SmartShuffleboard.put("AprilTag", "position-y", 0);
      }
    }
  }


  public int getTargetId() {
    return targetId;
  }
}
