// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperSubsystem extends SubsystemBase {
  public GripperSubsystem() {}
 
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void open(float speed) {

  }

  public void close(float speed) {

  }

  public void stop() {

  }
  
  public boolean getopenLimitSwitch() {
    return true;
  }
}