package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.apriltags.RobotPosition;
import frc.robot.utils.SmartShuffleboard;

/**
 * @see frc.robot.subsystems.PhotonCameraSubsystem 
 */
@Deprecated 
public class AprilTagPosition extends SubsystemBase {

  NetworkTableEntry tagIdLeftCam;
  NetworkTableEntry yRotationLeftCam;
  NetworkTableEntry horizontalOffsetLeftCam;
  NetworkTableEntry realDistanceLeftCam;
  NetworkTableEntry fieldXCoordinateLeft;
  NetworkTableEntry fieldYCoordinateLeft;
  NetworkTableEntry tagIdRightCam;
  NetworkTableEntry yRotationRightCam;
  NetworkTableEntry horizontalOffsetRightCam;
  NetworkTableEntry realDistanceRightCam;
  NetworkTableEntry fieldXCoordinateRight;
  NetworkTableEntry fieldYCoordinateRight;
  NetworkTableEntry fieldRotationLeft;
  NetworkTableEntry fieldRotationRight;

  private final Field2d robotField = new Field2d();

  NetworkTable tableLeft;
  NetworkTable tableRight;
  double distanceFromCamLeft;
  double distanceFromCamRight;

  public AprilTagPosition() {

    tableLeft = NetworkTableInstance.getDefault().getTable("apriltag").getSubTable("left");
    this.tagIdLeftCam = tableLeft.getEntry("tagId");
    this.yRotationLeftCam = tableLeft.getEntry("angleY");
    this.horizontalOffsetLeftCam = tableLeft.getEntry("horizontalOffset");
    this.realDistanceLeftCam = tableLeft.getEntry("realDistance");
    this.fieldXCoordinateLeft = tableLeft.getEntry("trueXCoordinate");
    this.fieldYCoordinateLeft = tableLeft.getEntry("trueYCoordinate");
    this.fieldRotationLeft = tableLeft.getEntry("fieldRotation");

    tableRight = NetworkTableInstance.getDefault().getTable("apriltag").getSubTable("right");
    this.tagIdRightCam = tableRight.getEntry("tagId");
    this.yRotationRightCam = tableRight.getEntry("angleY");
    this.horizontalOffsetRightCam = tableRight.getEntry("horizontalOffset");
    this.realDistanceRightCam = tableRight.getEntry("realDistance");
    this.fieldXCoordinateRight = tableRight.getEntry("trueXCoordinate");
    this.fieldYCoordinateRight = tableRight.getEntry("trueYCoordinate");
    this.fieldRotationRight = tableRight.getEntry("fieldRotation");

    SmartDashboard.putData(robotField);

  }

  public boolean isLeftCameraDetectingTag() {
    return tagIdLeftCam.getInteger(0L) != 0;
  }

  public boolean isRightCameraDetectingTag() {
    return tagIdRightCam.getInteger(0L) != 0;
  }

  public Pose2d getTagRelativePose2d() {
    RobotPosition position = getRobotPosition();
    if (position != null) {
      return new Pose2d(position.depthOffset, position.horizontalOffset, new Rotation2d(position.rotation));
    }
    return null;
  }

  public Pose2d getFieldRelativePose2d() {
    RobotPosition position = getRobotPosition();
    if (position != null) {
      return new Pose2d(position.fieldXPosition, position.fieldYPosition, new Rotation2d(position.fieldRotation));
    }
    return null;
  }

  private RobotPosition getLeftRobotPosition() {
    RobotPosition positionLeft = new RobotPosition();
    positionLeft.horizontalOffset = horizontalOffsetLeftCam.getDouble(0);
    positionLeft.depthOffset = realDistanceLeftCam.getDouble(0);
    positionLeft.rotation = yRotationLeftCam.getDouble(0);
    positionLeft.fieldXPosition = fieldXCoordinateLeft.getDouble(0);
    positionLeft.fieldYPosition = fieldYCoordinateLeft.getDouble(0);
    positionLeft.fieldRotation = fieldRotationLeft.getDouble(0);
    return positionLeft;
  }

  private RobotPosition getRightRobotPosition() {
    RobotPosition positionRight = new RobotPosition();
    positionRight.horizontalOffset = horizontalOffsetRightCam.getDouble(0);
    positionRight.depthOffset = realDistanceRightCam.getDouble(0);
    positionRight.rotation = yRotationRightCam.getDouble(0);
    positionRight.fieldXPosition = fieldXCoordinateRight.getDouble(0);
    positionRight.fieldYPosition = fieldYCoordinateRight.getDouble(0);
    positionRight.fieldRotation = fieldRotationRight.getDouble(0);
    return positionRight;
  }

  public RobotPosition getRobotPosition() {
    if (isLeftCameraDetectingTag() && !isRightCameraDetectingTag()) {
      return getLeftRobotPosition();
    }

    else if (isRightCameraDetectingTag() && !isLeftCameraDetectingTag()) {
      return getRightRobotPosition();
    } else if (isLeftCameraDetectingTag() && isRightCameraDetectingTag()) {
      distanceFromCamLeft = Math.sqrt((horizontalOffsetLeftCam.getDouble(0) * horizontalOffsetLeftCam.getDouble(0))
          + (realDistanceLeftCam.getDouble(0) * realDistanceLeftCam.getDouble(0)));
      distanceFromCamRight = Math.sqrt((horizontalOffsetRightCam.getDouble(0) * horizontalOffsetRightCam.getDouble(0))
          + (realDistanceRightCam.getDouble(0) * realDistanceRightCam.getDouble(0)));
      if (distanceFromCamLeft <= distanceFromCamRight) {
        return getLeftRobotPosition();
      } else {
        return getRightRobotPosition();
      }
    }
    return null;
  }

  @Override
  public void periodic() {
    if (Constants.APRILTAG_DEBUG == true) {
      RobotPosition position = getRobotPosition();
      if (position != null) {
        SmartShuffleboard.put("apriltag", "fieldX", position.fieldXPosition);
        SmartShuffleboard.put("apriltag", "fieldY", position.fieldYPosition);
        SmartShuffleboard.put("apriltag", "rotation", position.fieldRotation);
        SmartShuffleboard.put("apriltag", "tagRotation", position.rotation);
        robotField.setRobotPose(getFieldRelativePose2d());
      }
    }
  }

}
