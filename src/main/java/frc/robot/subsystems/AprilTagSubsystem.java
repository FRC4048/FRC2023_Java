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
  NetworkTableEntry xAxisLeftCam;
  NetworkTableEntry yAxisLeftCam;
  NetworkTableEntry zAxisLeftCam;
  NetworkTableEntry xRotationLeftCam;
  NetworkTableEntry yRotationLeftCam;
  NetworkTableEntry zRotatonLeftCam;
  NetworkTableEntry horizontalOffsetLeftCam;
  NetworkTableEntry realDistanceLeftCam;

  NetworkTableEntry tagIdRightCam;
  NetworkTableEntry xAxisRightCam;
  NetworkTableEntry yAxisRightCam;
  NetworkTableEntry zAxisRightCam;
  NetworkTableEntry xRotationRightCam;
  NetworkTableEntry yRotationRightCam;
  NetworkTableEntry zRotationRightCam;
  NetworkTableEntry horizontalOffsetRightCam;
  NetworkTableEntry realDistanceRightCam;

  NetworkTable tableLeft;
  NetworkTable tableRight;

  public AprilTagSubsystem() {

    tableLeft = NetworkTableInstance.getDefault().getTable("apriltag").getSubTable("left");
    this.tagIdLeftCam = tableLeft.getEntry("tagId");
    this.xAxisLeftCam = tableLeft.getEntry("xAxis");
    this.yAxisLeftCam = tableLeft.getEntry("yAxis");
    this.zAxisLeftCam = tableLeft.getEntry("zAxis");
    this.xRotationLeftCam = tableLeft.getEntry("angleX");
    this.yRotationLeftCam = tableLeft.getEntry("angleY");
    this.zRotatonLeftCam = tableLeft.getEntry("angleZ");
    this.horizontalOffsetLeftCam = tableLeft.getEntry("horizontalOffset");
    this.realDistanceLeftCam = tableLeft.getEntry("realDistance");

    tableRight = NetworkTableInstance.getDefault().getTable("apriltag").getSubTable("right");
    this.tagIdRightCam = tableRight.getEntry("tagId");
    this.xAxisRightCam = tableRight.getEntry("xAxis");
    this.yAxisRightCam = tableRight.getEntry("yAxis");
    this.zAxisRightCam = tableRight.getEntry("zAxis");
    this.xRotationRightCam = tableRight.getEntry("angleX");
    this.yRotationRightCam = tableRight.getEntry("angleY");
    this.zRotationRightCam = tableRight.getEntry("angleZ");
    this.horizontalOffsetRightCam = tableRight.getEntry("horizontalOffset");
    this.realDistanceRightCam = tableRight.getEntry("realDistance");

  }

  public Long getTagIdLeftCam() {
    return tagIdLeftCam.getInteger(0L);
  }

  public Double getxAxisLeftCam() {
    return xAxisLeftCam.getDouble(0.0);
  }

  public Double getyAxisLeftCam() {
    return yAxisLeftCam.getDouble(0.0);
  }

  public Double getzAxisLeftCam() {
    return zAxisLeftCam.getDouble(0.0);
  }

  public Double getxRotationLeftCam() {
    return xRotationLeftCam.getDouble(0.0);
  }

  public Double getyRotationLeftCam() {
    return yRotationLeftCam.getDouble(0.0);
  }

  public Double getzRotationLeftCam() {
    return zRotatonLeftCam.getDouble(0.0);
  }

  public Double getHorizontalOffsetLeftCam() {
    return horizontalOffsetLeftCam.getDouble(0.0);
  }

  public Double getRealDistanceLeftCam() {
    return realDistanceLeftCam.getDouble(0.0);
  }

  public Long getTagIdRightCam() {
    return tagIdRightCam.getInteger(0L);
  }

  public Double getxAxisRightCam() {
    return xAxisRightCam.getDouble(0.0);
  }

  public Double getyAxisRightCam() {
    return yAxisRightCam.getDouble(0.0);
  }

  public Double getzAxisRightCam() {
    return zAxisRightCam.getDouble(0.0);
  }

  public Double getxRotationRightCam() {
    return xRotationRightCam.getDouble(0.0);
  }

  public Double getyRotationRightCam() {
    return yRotationRightCam.getDouble(0.0);
  }

  public Double getzRotationRightCam() {
    return zRotationRightCam.getDouble(0.0);
  }

  public Double getHorizontalOffsetRightCam() {
    return horizontalOffsetRightCam.getDouble(0.0);
  }

  public Double getRealDistanceRightCam() {
    return realDistanceRightCam.getDouble(0.0);
  }

  @Override
  public void periodic() {
  }

  public Pose2d getPose2dLeftCam() {
    Pose2d pose2d = new Pose2d(new Translation2d(getxAxisLeftCam(), getzAxisLeftCam()),
        new Rotation2d(Math.toRadians(getyRotationLeftCam())));
    return pose2d;
  }

  public Pose2d getPose2dRightCam() {
    Pose2d pose2d = new Pose2d(new Translation2d(getxAxisRightCam(), getzAxisRightCam()),
        new Rotation2d(Math.toRadians(getyRotationRightCam())));
    return pose2d;
  }

}