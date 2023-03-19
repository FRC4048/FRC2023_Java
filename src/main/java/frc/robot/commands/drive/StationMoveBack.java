// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class StationMoveBack extends CommandBase {
  /** Creates a new MoveDistanceX. */
  private Drivetrain drivetrain;
  private double startPos;
  private double speed;
  private double startTime;
  
  public StationMoveBack(Drivetrain drivetrain, Double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    startPos = drivetrain.getPoseX();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(speed, 0.0, 0.0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(startPos - drivetrain.getPoseX()) > Constants.SUBSTATION_DRIVE_BACK_DISTANCE) || ((Timer.getFPGATimestamp() - startTime) > Constants.SUBSTATION_DRIVE_BACK_TIMEOUT);
  }
}
