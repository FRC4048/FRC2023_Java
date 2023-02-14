package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagSubsystem extends SubsystemBase {

  NetworkTableEntry tagId;
  NetworkTableEntry xAxis;
  NetworkTableEntry zAxis;
  NetworkTableEntry rotation;
  NetworkTable table;

  public AprilTagSubsystem() {

    table = NetworkTableInstance.getDefault().getTable("apriltag");
    this.tagId = table.getEntry("tagId");
    this.xAxis = table.getEntry("xAxis");
    this.zAxis = table.getEntry("zAxis");
    this.rotation = table.getEntry("rot");

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

  public double getRotation() {
    return rotation.getDouble(0.0);
  }

}
