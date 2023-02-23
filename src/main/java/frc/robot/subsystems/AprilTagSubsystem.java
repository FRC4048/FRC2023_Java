package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagSubsystem extends SubsystemBase {

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

  public AprilTagSubsystem() {

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

  public Long getTagId() {
    Long leftTagID = tagIdLeftCam.getInteger(0L);
    Long rightTagID = tagIdRightCam.getInteger(0L);
    if (leftTagID != 0) {
      return leftTagID;
    }
    else if (rightTagID != 0) {
      return tagIdRightCam.getInteger(0L);
    }
    else {
      return null;
    }
  }

  public Double getRotation() {
    Long leftTagID = tagIdLeftCam.getInteger(0L);
    Long rightTagID = tagIdRightCam.getInteger(0L);
    if (leftTagID != 0) {
      return yRotationLeftCam.getDouble(0);
    }
    else if (rightTagID != 0) {
      return yRotationRightCam.getDouble(0L);
    }
    else {
      return null;
    }
  }

  public Double getHorizontalOffset() {
    Long leftTagID = tagIdLeftCam.getInteger(0L);
    Long rightTagID = tagIdRightCam.getInteger(0L);
    if (leftTagID != 0) {
      return horizontalOffsetLeftCam.getDouble(0);
    }
    else if (rightTagID != 0) {
      return horizontalOffsetRightCam.getDouble(0L);
    }
    else {
      return null;
    }
  }

  public Double getDistance() {
    Long leftTagID = tagIdLeftCam.getInteger(0L);
    Long rightTagID = tagIdRightCam.getInteger(0L);
    if (leftTagID != 0) {
      return realDistanceLeftCam.getDouble(0);
    }
    else if (rightTagID != 0) {
      return realDistanceRightCam.getDouble(0L);
    }
    else {
      return null;
    }
  }

  @Override
  public void periodic() {
  }

  public Pose2d getPose2d() {
    Pose2d pose2d = new Pose2d(getDistance(), getHorizontalOffset(), new Rotation2d(Math.toRadians(getRotation())));
    return pose2d;
  }

}