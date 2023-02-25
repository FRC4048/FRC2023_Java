// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.SmartShuffleboard;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  Drivetrain drivetrain;
  double speedx, speedy;
  double filterx, filtery;
  public AutoBalance(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    filterx = drivetrain.getAccelX();
    filtery = drivetrain.getAccelY();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speedx = 0;
    speedy= 0;
    filterx = filterx + (filterx + drivetrain.getAccelX())/3;
    filtery = filtery + (filtery + drivetrain.getAccelY())/5;
    SmartShuffleboard.put("Auto Balance", "Filter x", drivetrain.getAccelX());
    SmartShuffleboard.put("Auto Balance", "Filter y", drivetrain.getAccelY());
    if (Math.abs(filterx) > 0.05) {
      speedx = Math.signum(filterx) * 0.1;
    }

    if (Math.abs(filtery) > 0.05) {
      speedy = Math.signum(filtery) * 0.1;
    }

    drivetrain.drive(speedx, 0, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(filterx) < 0.1 & Math.abs(filtery) < 0.1;
  }
}
