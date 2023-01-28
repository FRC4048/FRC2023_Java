// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SmartShuffleboard;

public class ExampleSubsystem extends SubsystemBase {

  private ADIS16470_IMU gyro;
  
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    gyro = new ADIS16470_IMU();
  }
  
  public void resetGyro() {
    gyro.reset();
    gyro.calibrate();
  }

  public double getAccelY() {
    return gyro.getAccelY();
  }

  public double getAccelX() {
    return gyro.getAccelX();
  }

  public double getAccelZ() {
    return gyro.getAccelZ();
  }

  public ADIS16470_IMU getGyro() {
    return gyro;
  }



  
  public CommandBase exampleMethodCommand() {
    return runOnce(
        () -> {
        });
  }

  
  public boolean exampleCondition() {
    return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartShuffleboard.put("Gyro", "Y accel", getAccelY());
    SmartShuffleboard.put("Gyro", "X accel", getAccelX());
    SmartShuffleboard.put("Gyro", "Z accel", getAccelZ());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
