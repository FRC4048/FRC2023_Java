// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class LockWheels extends CommandBase {
  /** Creates a new LockWheels. */
  Drivetrain drivetrain;
  double startTime;
  public LockWheels(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(0, 0, Constants.LOCK_WHEEL_ROT_SPEED, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > Constants.LOCK_WHEELS_TIMEOUT;
  }
}
