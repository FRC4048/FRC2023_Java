package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagSubsystem extends SubsystemBase {

  NetworkTableEntry tagId;
  NetworkTableEntry xAxis;
  NetworkTableEntry zAxis;
  NetworkTableEntry yRotation;
  NetworkTableEntry xRotation;
  NetworkTableEntry zRotaton;
  NetworkTable table;


  public AprilTagSubsystem() {

    table = NetworkTableInstance.getDefault().getTable("apriltag");
    this.tagId = table.getEntry("tagId");
    this.xAxis = table.getEntry("xAxis");
    this.zAxis = table.getEntry("zAxis");
    this.yRotation = table.getEntry("rot");
    this.xRotation = table.getEntry("angleX");
    this.zRotaton = table.getEntry("angleZ");
    
    
  }

  @Override
  public void periodic() {
  }

  public long getTagID() {
    return tagId.getInteger(0L);
  }

  public double getxAxis() {
    return xAxis.getDouble(0.0);
  }

  public double getzAxis() {
    return zAxis.getDouble(0.0);
  }

  public double getRotationY() {
    return yRotation.getDouble(0.0);
  }

  public double getRotationX() {
    return xRotation.getDouble(0.0);
  }

  public double getRotationZ() {
    return zRotaton.getDouble(0.0);
  }

  public Pose2d getPose2d() {
    Pose2d pose2d = new Pose2d(new Translation2d(getxAxis(), getzAxis()), new Rotation2d(getRotationY()));
    return pose2d;
  }

}
