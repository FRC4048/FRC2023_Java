// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class CrossPanel extends CommandBase {
  /** Creates a new DriveOverPanel. */
  private Drivetrain drivetrain;
  private boolean climbing;
  private double speed;
  private float climbDir;
  private boolean crossed;
  private boolean down;
  private double startTime;

  public CrossPanel(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbing = false;
    crossed = false;
    down = false;
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((Math.abs(drivetrain.getFilterRoll()) > 10) && !climbing) {
      climbing = true;
      climbDir = drivetrain.getFilterRoll();
    }
    
    if ((Math.signum(-drivetrain.getFilterRoll()) == Math.signum(climbDir)) && 
         Math.abs(drivetrain.getFilterRoll()) > 10) {
          climbing = false;
          crossed = true;
    }

    if (!down) {
      speed = 1;
    } else {
      speed = .6;
    }

    if ((drivetrain.getFilterRoll() < 7 && drivetrain.getFilterRoll() > -7) && climbing) {
      speed = 0.05;
      down = true;
    }

    drivetrain.drive(speed, 0, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (crossed && Math.abs(drivetrain.getFilterRoll()) < .7) || ((Timer.getFPGATimestamp() - startTime) > Constants.CHARGESTATION_TIMEOUT);
  }
}
