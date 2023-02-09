// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.limelight.LimeLightVision;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  NetworkTableEntry tv;
  NetworkTableEntry tx;
  NetworkTableEntry ta;
  NetworkTableEntry thor;
  NetworkTableEntry tvert;
  NetworkTableEntry tid;

  public Limelight() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ta = table.getEntry("ta");
    thor = table.getEntry("thor");
    tvert = table.getEntry("tvert");
    tid = table.getEntry("tid");
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartShuffleboard.put("apriltag", "tv", tv.getNumber(0.0));
    SmartShuffleboard.put("apriltag", "tx", tx.getNumber(0.0));
    SmartShuffleboard.put("apriltag", "ta", ta.getNumber(0.0));
    SmartShuffleboard.put("apriltag", "thor", thor.getNumber(0.0));
    SmartShuffleboard.put("apriltag", "tvert", tvert.getNumber(0.0));
    SmartShuffleboard.put("apriltag", "tid", tid.getNumber(0.0));

    
  }
}
