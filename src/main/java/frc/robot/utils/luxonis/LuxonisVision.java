// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.luxonis;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SmartShuffleboard;

public class LuxonisVision extends SubsystemBase {
  /** Creates a new LuxonisVision. */
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry tz;
  NetworkTableEntry tlabel;

  //The Luxonis publishes spatial coordinates where x and y are the frame and z is depth
  //The Odometry takes data with the field's x and y plane, with z as height
  //Keep this in mind when using odometry with the luxonis

  public LuxonisVision() {
    table = NetworkTableInstance.getDefault().getTable("Luxonis");
    tx = table.getEntry("x");
    ty = table.getEntry("y");
    tz = table.getEntry("z");
    tlabel = table.getEntry("label");
  }

  public double getObjectX() {
    return tx.getDouble(0.0);
  }

  public double getObjectY() {
    return ty.getDouble(0.0);
  }

  public double getObjectZ() {
    return tz.getDouble(0.0);
  }

  public String getObjectLabel() {
    return tlabel.getString("null");
  }

  public boolean hasTargetLock() {
    return tlabel != null;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartShuffleboard.put("Luxonis", "Object Label", getObjectLabel());
    SmartShuffleboard.put("Luxonis", "Object X", getObjectX());
    SmartShuffleboard.put("Luxonis", "Object Y", getObjectY());
    SmartShuffleboard.put("Luxonis", "Object Z", getObjectZ());

  }
}
