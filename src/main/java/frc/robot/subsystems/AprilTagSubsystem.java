package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    this.horizontalOffsetRightCam = tableRight.getEntry("horizontalOffset");
    this.realDistanceRightCam = tableRight.getEntry("realDistance");

  }

  public Long getTagId() {
    return tagIdLeftCam.getInteger(0L);
  }

  public Double getRotation() {
    double rotation = yRotationLeftCam.getDouble(0.0) + yRotationRightCam.getDouble(0.0);
    rotation = rotation / 2;
    return rotation;
  }

  public Double getHorizontalOffset() {
    double horizontalOffset = horizontalOffsetLeftCam.getDouble(0.0)+ horizontalOffsetRightCam.getDouble(0.0);
    horizontalOffset = horizontalOffset / 2;
    return horizontalOffset;
  }

  public Double getDistance() {
    double distance = realDistanceLeftCam.getDouble(0.0) + realDistanceRightCam.getDouble(0.0);
    distance = distance / 2;
    return distance;
  }

  @Override
  public void periodic() {
  }

  public Pose2d getPose2d() {
    Pose2d pose2d = new Pose2d(getDistance(), getHorizontalOffset(), new Rotation2d(Math.toRadians(getRotation())));
    return pose2d;
  }

}