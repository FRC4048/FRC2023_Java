// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TurnToDegrees extends CommandBase {
  /** Creates a new TurnToDegree. */
  Drivetrain drivetrain;
  double degrees;
  public TurnToDegrees(Drivetrain drivetrain, double degrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.degrees = degrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.turnToDegrees(degrees);
    drivetrain.setTurnToDegreeState(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
