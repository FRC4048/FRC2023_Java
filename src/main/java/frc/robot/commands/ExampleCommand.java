// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.utils.SmartShuffleboard;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ExampleCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExampleSubsystem subsystem;
  private boolean isGreater;
  
  public ExampleCommand(ExampleSubsystem subsystem) {
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    subsystem.resetGyro();
  }

  @Override
  public void execute() {
    if(subsystem.getAccelY() > 1) {
      isGreater = true;
    }
    if(subsystem.getAccelY() <= 1) {
      isGreater = false;
    }
    SmartShuffleboard.put("Gyro", "is greater?", isGreater);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
