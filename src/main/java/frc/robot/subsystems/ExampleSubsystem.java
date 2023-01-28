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

  public ADIS16470_IMU getGyro() {
    return gyro;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartShuffleboard.put("Gyro", "Yangle", getAccelY());
    SmartShuffleboard.put("Gyro", "Xangle", getAccelX());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
