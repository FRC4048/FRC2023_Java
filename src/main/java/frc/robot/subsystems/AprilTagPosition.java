package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AprilTagPosition {

  NetworkTableEntry tagIdLeftCam;
  NetworkTableEntry yRotationLeftCam;
  NetworkTableEntry horizontalOffsetLeftCam;
  NetworkTableEntry realDistanceLeftCam;

  NetworkTableEntry tagIdRightCam;
  NetworkTableEntry yRotationRightCam;
  NetworkTableEntry horizontalOffsetRightCam;
  NetworkTableEntry realDistanceRightCam;

  NetworkTable tableLeft;
  NetworkTable tableRight;

  public AprilTagPosition() {

    tableLeft = NetworkTableInstance.getDefault().getTable("apriltag").getSubTable("left");
    this.tagIdLeftCam = tableLeft.getEntry("tagId");
    this.yRotationLeftCam = tableLeft.getEntry("angleY");
    this.horizontalOffsetLeftCam = tableLeft.getEntry("horizontalOffset");
    this.realDistanceLeftCam = tableLeft.getEntry("realDistance");

    tableRight = NetworkTableInstance.getDefault().getTable("apriltag").getSubTable("right");
    this.tagIdRightCam = tableRight.getEntry("tagId");
    this.yRotationRightCam = tableRight.getEntry("angleY");
    this.horizontalOffsetRightCam = tableRight.getEntry("horizontalOffset");
    this.realDistanceRightCam = tableRight.getEntry("realDistance");
  }

  public boolean isLeftCameraDetectingTag() {
    return tagIdLeftCam.getInteger(0L) != 0;
  }

  public boolean isRightCameraDetectingTag() {
    return tagIdRightCam.getInteger(0L) != 0;
  }
  
  public Long getTagId() {
    long leftTagID = tagIdLeftCam.getInteger(0);
    long rightTagID = tagIdRightCam.getInteger(0);
    if (leftTagID != 0) {
      return leftTagID;
    }
    else if (rightTagID != 0) {
      return rightTagID;
    }
    else {
      return null;
    }
  }

  public Double getRotation() {
    if (isLeftCameraDetectingTag()) {
      return yRotationLeftCam.getDouble(0);
    } else if (isRightCameraDetectingTag()) {
      return yRotationRightCam.getDouble(0);
    }
    return null;
  }

  public Double getHorizontalOffset() {
    if (isLeftCameraDetectingTag()) {
      return horizontalOffsetLeftCam.getDouble(0);
    }
    else if (isRightCameraDetectingTag()) {
      return horizontalOffsetRightCam.getDouble(0L);
    }
    else {
      return null;
    }
  }

  public Double getDistance() {
    if (isLeftCameraDetectingTag()) {
      return realDistanceLeftCam.getDouble(0);
    }
    else if (isRightCameraDetectingTag()) {
      return realDistanceRightCam.getDouble(0L);
    }
    else {
      return null;
    }
  }

  public Pose2d getPose2d() {
    if (isLeftCameraDetectingTag() || isRightCameraDetectingTag()) {
      return new Pose2d(getDistance(), getHorizontalOffset(), new Rotation2d(getRotation()));
    }
    return null;
  }
}